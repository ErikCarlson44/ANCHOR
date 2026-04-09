import React from 'react'
import { motion } from 'framer-motion'
import './Telemetry.css'

function fmt(n, digits, fallback = '—') {
  const x = Number(n)
  return Number.isFinite(x) ? x.toFixed(digits) : fallback
}

function Telemetry({ data }) {
  const obstacles = Array.isArray(data.obstacles) ? data.obstacles : []
  const batN = Number(data.battery)
  const batteryColor =
    Number.isFinite(batN) && batN > 50
      ? 'var(--success)'
      : Number.isFinite(batN) && batN > 25
        ? 'var(--warning)'
        : 'var(--danger)'

  return (
    <div className="telemetry-panel">
      <div className="panel-header">
        <span className="panel-icon">◈</span>
        <h2>TELEMETRY</h2>
      </div>

      <div className="telemetry-content">
        <div className="telemetry-section">
          <div className="section-label">GPS POSITION</div>
          <div className="gps-grid">
            <div className="data-item">
              <span className="data-key">LAT</span>
              <span className="data-value">{fmt(data.latitude, 6)}°</span>
            </div>
            <div className="data-item">
              <span className="data-key">LON</span>
              <span className="data-value">{fmt(data.longitude, 6)}°</span>
            </div>
          </div>
        </div>

        <div className="telemetry-section">
          <div className="section-label">NAVIGATION</div>
          <div className="nav-grid">
            <div className="data-item large">
              <span className="data-key">HEADING</span>
              <div className="heading-display">
                <span className="heading-value">
                  {Number.isFinite(Number(data.heading))
                    ? String(Math.round(Number(data.heading))).padStart(3, '0')
                    : '---'}
                </span>
                <span className="heading-unit">°</span>
                <CompassIndicator heading={data.heading} />
              </div>
            </div>
            <div className="data-item large">
              <span className="data-key">SPEED</span>
              <div className="speed-display">
                <span className="speed-value">{fmt(data.speed, 1)}</span>
                <span className="speed-unit">kts</span>
              </div>
            </div>
          </div>
        </div>

        <div className="telemetry-section">
          <div className="section-label">POWER SYSTEM</div>
          <div className="battery-display">
            <div className="battery-header">
              <span className="battery-icon">⚡</span>
              <span className="battery-label">BATTERY</span>
              <span className="battery-percent" style={{ color: batteryColor }}>
                {fmt(data.battery, 0)}%
              </span>
            </div>
            <div className="battery-bar-container">
              <motion.div
                className="battery-bar"
                initial={{ width: 0 }}
                animate={{
                  width: `${Number.isFinite(Number(data.battery)) ? Math.min(100, Math.max(0, Number(data.battery))) : 100}%`,
                }}
                style={{ background: batteryColor }}
                transition={{ duration: 0.3 }}
              />
              <div className="battery-segments">
                {[...Array(10)].map((_, i) => (
                  <div key={i} className="segment" />
                ))}
              </div>
            </div>
          </div>
        </div>

        <div className="telemetry-section">
          <div className="section-label">RADAR STATUS</div>
          <div className="obstacle-display">
            <div className="obstacle-count">
              <span className="obstacle-number">{obstacles.length}</span>
              <span className="obstacle-label">OBJECTS DETECTED</span>
            </div>
            <div className="radar-status-icon">
              <div className="radar-sweep" />
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

function CompassIndicator({ heading }) {
  const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
  const h = Number(heading)
  const index = Number.isFinite(h) ? Math.round(h / 45) % 8 : 0

  return (
    <div className="compass-indicator">
      <span className="compass-direction">{directions[index]}</span>
    </div>
  )
}

export default Telemetry
