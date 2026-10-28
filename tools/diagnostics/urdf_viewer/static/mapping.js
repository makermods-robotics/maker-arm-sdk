export const rad = deg => deg * Math.PI / 180;
export const deg = radian => radian * 180 / Math.PI;

// Leader degrees -> follower degrees per degree. LeRobot-style mappings fold the scale into
// `direction`; older files carried a separate `scale`.
export const mappingGain = mapping => mapping.direction * (mapping.scale ?? 1);

// Absolute SDK targets are useful telemetry, but are NOT calibrated URDF angles. Mirrors
// maker_arm.mapping.JointMapper: unwrap whole servo turns around the follower range expressed
// in leader degrees, scale by direction, clip to range_deg. No offset: both arms share a zero.
export function sdkTarget(rawDeg, mapping) {
  const [lo, hi] = mapping.range_deg;
  const a = lo / mapping.direction, b = hi / mapping.direction;
  const center = (a + b) / 2;
  const unwrapped = rawDeg - Math.round((rawDeg - center) / 360) * 360;
  return rad(Math.max(lo, Math.min(hi, unwrapped * mapping.direction)));
}

export function relativeAngle(rawDeg, restDeg, referenceRad, sign, mapping, mode = 'sdk') {
  const gain = mode === 'sdk' ? mappingGain(mapping) : 1;
  return referenceRad + sign * gain * rad(rawDeg - restDeg);
}

export function withinLimits(q, limit) {
  return Number.isFinite(q) && q >= limit.lower - 1e-7 && q <= limit.upper + 1e-7;
}

export function freshSnapshot(state, elapsedMs = 0, maxAgeMs = 500) {
  return state.readings.map(r => ({...r,
    fresh: r.fresh && r.age_ms !== null && r.age_ms + elapsedMs <= maxAgeMs,
  }));
}

// Endpoint interpolation is only a visual proxy for the unmeasured linkage curve.
export function gripperGap(rawDeg, restDeg, mapping, calibration, reverse = false) {
  const span = calibration.commanded_open_encoder_rad - calibration.closed_encoder_rad;
  const motorDelta = mappingGain(mapping) * rad(rawDeg-restDeg) * (reverse ? -1 : 1);
  const requested = .104825 * motorDelta / span;
  return {requested, applied: Math.min(.104825, Math.max(0, requested))};
}
