import array, gc

N_PIXELS = 768         # 32 × 24 sensor array
TEMP_MIN = 10.0        # °C — valid sensor range used for temperature normalisation
TEMP_MAX = 40.0

# uint16 image format:
#   Values 0–65534 linearly encode temperatures in [TEMP_MIN, TEMP_MAX]:
#     temp = value / 65535.0 * (TEMP_MAX - TEMP_MIN) + TEMP_MIN
#   Value 65535 (NULL_PIXEL) marks an invalid or stuck pixel and is
#   excluded from all image processing. The sensor library identifies
#   stuck pixels as -273.15 °C in the raw float frame.
NULL_PIXEL = 65535

_NAN = float('nan')


def build_ignore_mask(zones):
    mask = bytearray(N_PIXELS)
    if zones:
        for i in range(N_PIXELS):
            col, row = i % 32, i // 32
            for z in zones:
                if z[0] <= col <= z[2] and z[1] <= row <= z[3]:
                    mask[i] = 1
                    break
    return mask


def encode_temps(raw):
    # Encode a float frame (°C) as a uint16 reference image (format above).
    span = TEMP_MAX - TEMP_MIN
    out = array.array('H', (0 for _ in range(N_PIXELS)))
    for i, v in enumerate(raw):
        if v != v or v < -200.0:
            out[i] = NULL_PIXEL
        else:
            out[i] = max(0, min(65534, round((v - TEMP_MIN) / span * 65535)))
    return out


def score(current_frame, raw_b, mask=None):
    # Blob score (°C²): take the per-pixel difference between the current frame
    # and the reference raw_b, remove its mean, smooth it with PASSES passes of
    # a 3×3 Gaussian, and return the largest squared value.
    PASSES = 4
    W, H = 32, 24
    span = TEMP_MAX - TEMP_MIN
    # Current-frame validity is already resolved by read_sensor (invalid pixels are
    # NaN). Here a pixel is dropped when the current reading is NaN, the reference
    # pixel is NULL, or the pixel is masked out.
    buf = array.array('f', (
        _NAN if (
            current_frame[i] != current_frame[i]
            or raw_b[i] == NULL_PIXEL
            or (mask is not None and mask[i])
        )
        else current_frame[i] - (raw_b[i] * span / 65535.0 + TEMP_MIN)
        for i in range(N_PIXELS)
    ))
    total = cnt = 0
    for v in buf:
        if v == v:  # NaN != NaN
            total += v
            cnt += 1
    mean = total / cnt if cnt else 0.0
    for i in range(N_PIXELS):
        if buf[i] == buf[i]:
            buf[i] -= mean
    tmp = array.array('f', [0.0] * N_PIXELS)
    for _ in range(PASSES):
        for row in range(H):
            for col in range(W):
                idx = row * W + col
                c = buf[idx]
                if c != c:
                    tmp[idx] = _NAN
                    continue
                lv = buf[idx - 1] if col > 0     else c
                rv = buf[idx + 1] if col < W - 1 else c
                lv = c if lv != lv else lv  # null neighbour → substitute center (like edge clamp)
                rv = c if rv != rv else rv
                tmp[idx] = (lv + c * 2.0 + rv) * 0.25
        for row in range(H):
            for col in range(W):
                idx = row * W + col
                c = tmp[idx]
                if c != c:
                    buf[idx] = _NAN
                    continue
                uv = tmp[idx - W] if row > 0     else c
                dv = tmp[idx + W] if row < H - 1 else c
                uv = c if uv != uv else uv
                dv = c if dv != dv else dv
                buf[idx] = (uv + c * 2.0 + dv) * 0.25
    del tmp
    best = 0.0
    for v in buf:
        if v == v:
            sq = v * v
            if sq > best:
                best = sq
    return best


def read_sensor(mlx, frame, slot_raw, slot_matched, trigger_thresholds, reset_thresholds, mask=None):
    # frame is an array.array('f') filled in-place by mlx.get_frame().
    # slot_raw holds each slot's reference image (see encode_temps), or None if unset.
    # Returns (presence, matched, slot_score, slot_matched, valid_px, healthy).
    # When healthy is False the frame was rejected as corrupt: presence/matched/
    # slot_score are None and the caller should hold its previous values. valid_px
    # is still reported so the glitch stays visible; slot_matched is echoed
    # unchanged so hysteresis state survives the rejected frame.
    gc.collect()
    # get_frame() reads both subpages (checkerboard halves) of the frame. It raises
    # if the second one never arrives or the I2C bus fails; that is left to the
    # caller, which holds its previous values.
    mlx.get_frame(frame)
    for lst in (mlx.broken_pixels, mlx.outlier_pixels):
        for i in lst:
            frame[i] = _NAN
    # Own frame validity here so downstream scoring sees a single signal (NaN):
    # invalidate NaN and out-of-range readings (the latter also catches the
    # -273.15 stuck-pixel marker) instead of clamping them. valid_px is a
    # sensor-health metric — a glitched/uniform/stale frame drops it well below
    # the usual ~760, whereas clamping would have hidden the glitch as a flat scene.
    valid_px = 0
    for i in range(N_PIXELS):
        v = frame[i]
        if v == v and TEMP_MIN <= v <= TEMP_MAX:
            valid_px += 1
        else:
            frame[i] = _NAN

    # ── Frame-validity gate (reject-and-hold) ─────────────────────────
    # Ta (ambient) and VDD feed every pixel's temperature calc, so a corrupted
    # read of either flattens the whole frame (valid_px collapses to ~0). Reject
    # on physically-impossible values — indoor ambient and the sensor supply
    # rail — rather than an arbitrary NaN count. On rejection the caller holds
    # its previous scores; only valid_px is updated so the glitch stays visible.
    try:
        ta  = mlx._get_ta()
        vdd = mlx._get_vdd()
    except Exception:
        ta, vdd = -999.0, 0.0
    if not (-20.0 <= ta <= 80.0) or not (2.4 <= vdd <= 3.6):  # datasheet operating ranges
        print('read_sensor: reject frame ta=%.1f vdd=%.2f valid_px=%d' % (ta, vdd, valid_px))
        return None, None, None, slot_matched, valid_px, False

    matched = []
    new_slot_matched = list(slot_matched)
    slot_score_out = [None, None, None]
    any_slot = any(r is not None for r in slot_raw)
    for i, ref in enumerate(slot_raw):
        if ref is not None:
            m = score(frame, ref, mask)
            slot_score_out[i] = round(m, 4)
            if new_slot_matched[i]:
                if m > trigger_thresholds[i]:
                    new_slot_matched[i] = False
            else:
                if m < reset_thresholds[i]:
                    new_slot_matched[i] = True
            if new_slot_matched[i]:
                matched.append(i)
        else:
            new_slot_matched[i] = False
    presence = any_slot and len(matched) == 0
    return presence, matched, slot_score_out, new_slot_matched, valid_px, True
