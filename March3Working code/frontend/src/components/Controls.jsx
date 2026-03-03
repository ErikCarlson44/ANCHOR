import React from 'react'
import { motion } from 'framer-motion'
import './Controls.css'

function Controls({ throttle, steering, keysPressed, throttleLimit, onThrottleLimitChange }) {
  const isKeyActive = (key) => keysPressed.has(key.toLowerCase())
  
  const throttleLimits = [10, 25, 50, 75, 100]
  
  return (
    <div className="controls-panel">
      <div className="panel-header">
        <span className="panel-icon">◇</span>
        <h2>CONTROLS</h2>
      </div>
      
      <div className="controls-content">
        {/* Keyboard layout */}
        <div className="keyboard-layout">
          <div className="key-row">
            <Key label="W" active={isKeyActive('w')} />
          </div>
          <div className="key-row">
            <Key label="A" active={isKeyActive('a')} />
            <Key label="S" active={isKeyActive('s')} />
            <Key label="D" active={isKeyActive('d')} />
          </div>
          <div className="key-row">
            <Key label="SPACE" wide active={isKeyActive(' ')} />
          </div>
        </div>
        
        {/* Control legend */}
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
        </div>
        
        {/* Throttle Limit Selector */}
        <div className="throttle-limit-section">
          <div className="gauge-label">MAX THROTTLE</div>
          <div className="throttle-limit-buttons">
            {throttleLimits.map(limit => (
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
        
        {/* Throttle & Steering Gauges */}
        <div className="gauges">
          <div className="gauge-container">
            <div className="gauge-label">THROTTLE</div>
            <div className="gauge vertical">
              <div className="gauge-track">
                {/* Limit indicator line */}
                <div 
                  className="throttle-limit-line"
                  style={{ bottom: `${throttleLimit}%` }}
                />
                <motion.div 
                  className="gauge-fill throttle"
                  style={{ 
                    height: `${throttle * 100}%`,
                    bottom: 0,
                    background: throttle * 100 >= throttleLimit ? 'var(--warning)' : 'var(--success)'
                  }}
                  animate={{ height: `${throttle * 100}%` }}
                  transition={{ duration: 0.1 }}
                />
              </div>
              <div className="gauge-value">
                {(throttle * 100).toFixed(0)}% <span className="limit-indicator">/ {throttleLimit}%</span>
              </div>
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

function Key({ label, active, wide }) {
  return (
    <motion.div 
      className={`key ${active ? 'active' : ''} ${wide ? 'wide' : ''}`}
      animate={{ 
        scale: active ? 0.95 : 1,
        boxShadow: active 
          ? '0 0 20px var(--accent-primary), inset 0 0 20px rgba(0, 255, 204, 0.2)'
          : '0 2px 8px rgba(0, 0, 0, 0.3)'
      }}
      transition={{ duration: 0.1 }}
    >
      {label}
    </motion.div>
  )
}

export default Controls

