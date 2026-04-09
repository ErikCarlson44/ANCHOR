/**
 * Coerce backend / Pico-shaped payloads into the shape the UI expects.
 * Prevents blank screen when JSON.parse succeeds but fields are missing or non-finite.
 */
export function normalizeTelemetry(raw) {
  const num = (v, fallback = 0) => {
    const x = Number(v)
    return Number.isFinite(x) ? x : fallback
  }

  let obs = raw?.obstacles ?? raw?.obs
  if (!Array.isArray(obs)) obs = []

  const obstacles = obs.map((o) => {
    if (Array.isArray(o) && o.length >= 2) {
      return {
        distance: num(o[0], 0),
        angle: num(o[1], 0),
        size: num(o[2], 1),
      }
    }
    if (o && typeof o === 'object') {
      return {
        distance: num(o.distance, 0),
        angle: num(o.angle, 0),
        size: num(o.size, 1),
      }
    }
    return { distance: 0, angle: 0, size: 1 }
  })

  return {
    latitude: num(raw?.latitude ?? raw?.lat, 32.7872),
    longitude: num(raw?.longitude ?? raw?.lon, -117.235),
    heading: num(raw?.heading ?? raw?.hdg, 0),
    speed: num(raw?.speed ?? raw?.spd, 0),
    battery: num(raw?.battery ?? raw?.bat, 100),
    satellites: Math.max(0, Math.floor(num(raw?.satellites ?? raw?.sats, 0))),
    obstacles,
    timestamp: num(raw?.timestamp, Date.now()) || Date.now(),
  }
}
