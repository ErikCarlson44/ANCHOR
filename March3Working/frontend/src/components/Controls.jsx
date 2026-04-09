import React from 'react'
import { motion } from 'framer-motion'
import './Controls.css'

function Controls({
  throttle,
  steering,
  keysPressed,
  throttleLimit,
  onThrottleLimitChange,
  onEmergencyStop,
  onThrottleNudge,
  throttleStep = 0.05,
}) {
  const isKeyActive = (key) => keysPressed.has(key === ' ' ? ' ' : key.toLowerCase())

  const throttleLimits = [10, 25, 50, 75, 100]
  const stepPct = Math.round(throttleStep * 100)
  // Pico boat ESC mapping (code.py / throttle_test): 1000 µs stop → 2000 µs full
  const escPulseUs = Math.round(1000 + 1000 * Math.min(1, Math.max(0, throttle)))

  return (
    <div className="controls-panel">
      <div className="panel-header">
        <span className="panel-icon">◇</span>
        <h2>CONTROLS</h2>
      </div>

      <div className="controls-content">
        <div className="keyboard-layout">
          <div className="key-row">
            <Key label="W" active={isKeyActive('w')} keysPressed={keysPressed} bindKey="w" />
          </div>
          <div className="key-row">
            <Key label="A" active={isKeyActive('a')} keysPressed={keysPressed} bindKey="a" />
            <Key label="S" active={isKeyActive('s')} keysPressed={keysPressed} bindKey="s" />
            <Key label="D" active={isKeyActive('d')} keysPressed={keysPressed} bindKey="d" />
          </div>
          <div className="key-row">
            <Key
              label="SPACE"
              wide
              active={isKeyActive(' ')}
              onPointerDown={(e) => {
                e.preventDefault()
                onEmergencyStop?.()
              }}
            />
          </div>
        </div>
        <p className="controls-hint">
          Throttle matches bench test: <strong>±{stepPct}%</strong> per tick while W/S held; level{' '}
          <strong>stays</strong> when you release (use S or Space/X to reduce). Hold{' '}
          <strong>W S A D</strong> or the on-screen keys. Pico ESC: <strong>1000–2000 µs</strong> pulse
          (GP14). Connect COM in Connection (close Thonny first).
        </p>

        {onThrottleNudge && (
          <div className="throttle-step-row">
            <button
              type="button"
              className="throttle-step-btn"
              onClick={() => onThrottleNudge(-throttleStep)}
            >
              −{stepPct}%
            </button>
            <button
              type="button"
              className="throttle-step-btn"
              onClick={() => onThrottleNudge(throttleStep)}
            >
              +{stepPct}%
            </button>
          </div>
        )}

        <div className="control-legend">
          <div className="legend-item">
            <span className="legend-key">W</span>
            <span className="legend-action">Throttle Up</span>
          </div>
          <div className="legend-item">
            <span className="legend-key">S</span>
            <span className="legend-action">Throttle Down</span>
          </div>
          <div className="legend-item">
            <span className="legend-key">A</span>
            <span className="legend-action">Rudder Left</span>
          </div>
          <div className="legend-item">
            <span className="legend-key">D</span>
            <span className="legend-action">Rudder Right</span>
          </div>
          <div className="legend-item">
            <span className="legend-key">SPACE</span>
            <span className="legend-action">All Stop</span>
          </div>
          <div className="legend-item">
            <span className="legend-key">X</span>
            <span className="legend-action">Stop throttle & rudder</span>
          </div>
        </div>

        <div className="throttle-limit-section">
          <div className="gauge-label">MAX THROTTLE</div>
          <div className="throttle-limit-buttons">
            {throttleLimits.map((limit) => (
              <button
                key={limit}
                className={`limit-btn ${throttleLimit === limit ? 'active' : ''}`}
                onClick={() => onThrottleLimitChange(limit)}
              >
                {limit}%
              </button>
            ))}
          </div>
        </div>

        <div className="gauges">
          <div className="gauge-container">
            <div className="gauge-label">THROTTLE</div>
            <div className="gauge vertical">
              <div className="gauge-track">
                <div
                  className="throttle-limit-line"
                  style={{ bottom: `${throttleLimit}%` }}
                />
                <motion.div
                  className="gauge-fill throttle"
                  style={{
                    height: `${throttle * 100}%`,
                    bottom: 0,
                    background:
                      throttle * 100 >= throttleLimit ? 'var(--warning)' : 'var(--success)',
                  }}
                  animate={{ height: `${throttle * 100}%` }}
                  transition={{ duration: 0.1 }}
                />
              </div>
              <div className="gauge-value">
                {(throttle * 100).toFixed(0)}%{' '}
                <span className="limit-indicator">/ {throttleLimit}%</span>
              </div>
              <div className="esc-pulse-hint">ESC ≈ {escPulseUs} µs</div>
            </div>
          </div>

          <div className="gauge-container">
            <div className="gauge-label">RUDDER</div>
            <div className="rudder-display">
              <div className={`rudder-position left ${steering < 0 ? 'active' : ''}`}>
                ◄ LEFT
              </div>
              <div className={`rudder-position center ${steering === 0 ? 'active' : ''}`}>
                CENTER
              </div>
              <div className={`rudder-position right ${steering > 0 ? 'active' : ''}`}>
                RIGHT ►
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

function Key({ label, active, wide, keysPressed, bindKey, onPointerDown }) {
  const pointerBind =
    bindKey && keysPressed
      ? {
          onPointerDown: (e) => {
            e.preventDefault()
            keysPressed.add(bindKey)
          },
          onPointerUp: (e) => {
            e.preventDefault()
            keysPressed.delete(bindKey)
          },
          onPointerCancel: () => keysPressed.delete(bindKey),
          onPointerLeave: () => keysPressed.delete(bindKey),
        }
      : {}
  const extraDown = onPointerDown
    ? {
        onPointerDown: (e) => {
          onPointerDown(e)
        },
      }
    : {}
  return (
    <motion.div
      className={`key ${active ? 'active' : ''} ${wide ? 'wide' : ''}`}
      style={{
        touchAction: 'none',
        userSelect: 'none',
        cursor: bindKey || onPointerDown ? 'pointer' : 'default',
      }}
      animate={{
        scale: active ? 0.95 : 1,
        boxShadow: active
          ? '0 0 20px var(--accent-primary), inset 0 0 20px rgba(0, 255, 204, 0.2)'
          : '0 2px 8px rgba(0, 0, 0, 0.3)',
      }}
      transition={{ duration: 0.1 }}
      {...pointerBind}
      {...extraDown}
    >
      {label}
    </motion.div>
  )
}

export default Controls
