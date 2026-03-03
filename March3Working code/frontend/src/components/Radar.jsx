import React, { useRef, useEffect, useState, useCallback } from 'react'
import './Radar.css'

function Radar({ heading, obstacles }) {
  const canvasRef = useRef(null)
  const sweepAngleRef = useRef(0)
  const targetsRef = useRef({}) // Key: unique id, Value: {dist, angle, timestamp, size}
  const animationRef = useRef(null)
  
  const [maxRange, setMaxRange] = useState(8) // meters (adjustable 1-20m)
  
  // Target lifetime in seconds
  const TARGET_LIFETIME = 2.0
  
  // Convert polar coordinates (distance, angle) to canvas x,y
  const polarToXY = useCallback((dist, angle, center, radius, maxRangeValue) => {
    // Normalize distance to radius
    const normalizedDist = (dist / maxRangeValue) * radius
    // Convert angle: 0° = up (North), 90° = right (East), etc.
    const rad = (angle - 90) * (Math.PI / 180)
    const x = center + normalizedDist * Math.cos(rad)
    const y = center + normalizedDist * Math.sin(rad)
    return { x, y }
  }, [])
  
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    
    const ctx = canvas.getContext('2d')
    const size = canvas.width
    const center = size / 2
    const radius = size / 2 - 35
    
    // Colors matching the working display
    const COLORS = {
      green: '#00ff00',
      darkGreen: '#006400',
      brightGreen: '#00ff64',
      cyan: '#00ffff',
      red: '#ff4444',
      white: '#ffffff',
      gray: '#888888'
    }
    
    const draw = () => {
      const now = Date.now() / 1000 // Current time in seconds
      
      // Clear canvas
      ctx.clearRect(0, 0, size, size)
      
      // Draw background
      drawBackground(ctx, center, radius)
      
      // Draw range circles with labels
      drawRangeCircles(ctx, center, radius)
      
      // Draw angle lines
      drawAngleLines(ctx, center, radius)
      
      // Draw sweep line with trail
      drawSweep(ctx, center, radius)
      
      // Update and draw targets
      updateTargets(now)
      drawTargets(ctx, center, radius, now)
      
      // Draw boat indicator at center
      drawBoat(ctx, center)
      
      // Draw heading and labels
      drawLabels(ctx, center, radius)
      
      // Update sweep angle (visual animation)
      sweepAngleRef.current = (sweepAngleRef.current + 3) % 360
      
      animationRef.current = requestAnimationFrame(draw)
    }
    
    const drawBackground = (ctx, center, radius) => {
      // Dark gradient background
      const gradient = ctx.createRadialGradient(center, center, 0, center, center, radius)
      gradient.addColorStop(0, '#0a1a15')
      gradient.addColorStop(1, '#050a08')
      
      ctx.beginPath()
      ctx.arc(center, center, radius, 0, Math.PI * 2)
      ctx.fillStyle = gradient
      ctx.fill()
      
      // Outer ring
      ctx.strokeStyle = COLORS.darkGreen
      ctx.lineWidth = 2
      ctx.stroke()
    }
    
    const drawRangeCircles = (ctx, center, radius) => {
      ctx.strokeStyle = COLORS.darkGreen
      ctx.lineWidth = 1
      ctx.font = '9px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.darkGreen
      
      // Draw range circles based on maxRange
      const numCircles = Math.min(Math.ceil(maxRange), 8)
      const rangeStep = maxRange / numCircles
      
      for (let i = 1; i <= numCircles; i++) {
        const rangeValue = rangeStep * i
        const r = (radius / numCircles) * i
        
        ctx.beginPath()
        ctx.arc(center, center, r, 0, Math.PI * 2)
        ctx.stroke()
        
        // Range label
        ctx.fillText(`${rangeValue.toFixed(0)}m`, center + 4, center - r + 12)
      }
    }
    
    const drawAngleLines = (ctx, center, radius) => {
      // Draw lines every 30 degrees
      for (let angle = 0; angle < 360; angle += 30) {
        const isCardinal = angle % 90 === 0
        const { x, y } = polarToXY(maxRange * 1.05, angle, center, radius, maxRange)
        
        ctx.beginPath()
        ctx.moveTo(center, center)
        ctx.lineTo(
          center + radius * Math.cos((angle - 90) * Math.PI / 180),
          center + radius * Math.sin((angle - 90) * Math.PI / 180)
        )
        ctx.strokeStyle = isCardinal ? COLORS.green : COLORS.darkGreen
        ctx.lineWidth = isCardinal ? 1.5 : 0.5
        ctx.stroke()
      }
      
      // Cardinal direction labels
      ctx.font = 'bold 11px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.darkGreen
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      
      const labelDist = radius + 15
      ctx.fillText('N', center, center - labelDist)
      ctx.fillText('S', center, center + labelDist)
      ctx.fillText('E', center + labelDist, center)
      ctx.fillText('W', center - labelDist, center)
    }
    
    const drawSweep = (ctx, center, radius) => {
      const sweepAngle = sweepAngleRef.current
      
      // Sweep trail (fading lines behind the sweep)
      for (let i = 1; i < 20; i++) {
        const trailAngle = (sweepAngle - i * 2 + 360) % 360
        const rad = (trailAngle - 90) * (Math.PI / 180)
        const intensity = Math.max(0, 100 - i * 5)
        
        ctx.beginPath()
        ctx.moveTo(center, center)
        ctx.lineTo(
          center + radius * Math.cos(rad),
          center + radius * Math.sin(rad)
        )
        ctx.strokeStyle = `rgb(0, ${intensity}, 0)`
        ctx.lineWidth = Math.max(1, 2 - i * 0.1)
        ctx.stroke()
      }
      
      // Main sweep line
      const sweepRad = (sweepAngle - 90) * (Math.PI / 180)
      ctx.beginPath()
      ctx.moveTo(center, center)
      ctx.lineTo(
        center + radius * Math.cos(sweepRad),
        center + radius * Math.sin(sweepRad)
      )
      ctx.strokeStyle = COLORS.brightGreen
      ctx.lineWidth = 2
      ctx.shadowColor = COLORS.brightGreen
      ctx.shadowBlur = 8
      ctx.stroke()
      ctx.shadowBlur = 0
    }
    
    const updateTargets = (now) => {
      // Add new obstacles from props
      obstacles.forEach((obs, idx) => {
        if (obs.distance > 0 && obs.distance <= maxRange) {
          // Create unique key based on approximate position
          const angleKey = Math.round(obs.angle / 5) * 5 // Round to nearest 5 degrees
          const distKey = Math.round(obs.distance * 10) // Round to nearest 0.1m
          const key = `${angleKey}_${distKey}_${idx}`
          
          targetsRef.current[key] = {
            dist: obs.distance,
            angle: obs.angle,
            size: obs.size || 1,
            timestamp: now
          }
        }
      })
      
      // Remove expired targets
      Object.keys(targetsRef.current).forEach(key => {
        const target = targetsRef.current[key]
        if (now - target.timestamp > TARGET_LIFETIME) {
          delete targetsRef.current[key]
        }
      })
    }
    
    const drawTargets = (ctx, center, radius, now) => {
      Object.values(targetsRef.current).forEach(target => {
        const age = now - target.timestamp
        if (age > TARGET_LIFETIME) return
        
        // Calculate fade (1.0 = new, 0.0 = about to disappear)
        const fade = 1.0 - (age / TARGET_LIFETIME)
        
        // Convert to screen coordinates
        const { x, y } = polarToXY(target.dist, target.angle, center, radius, maxRange)
        
        // Target size based on obstacle size and fade
        const baseSize = 5 + target.size * 2
        const displaySize = baseSize * (0.6 + fade * 0.4)
        
        // Glow effect
        const glowGradient = ctx.createRadialGradient(x, y, 0, x, y, displaySize * 2.5)
        glowGradient.addColorStop(0, `rgba(255, 80, 80, ${fade * 0.6})`)
        glowGradient.addColorStop(1, 'rgba(255, 80, 80, 0)')
        
        ctx.beginPath()
        ctx.arc(x, y, displaySize * 2.5, 0, Math.PI * 2)
        ctx.fillStyle = glowGradient
        ctx.fill()
        
        // Core target dot
        const r = Math.floor(255 * fade)
        const g = Math.floor(80 * fade)
        const b = Math.floor(80 * fade)
        
        ctx.beginPath()
        ctx.arc(x, y, displaySize, 0, Math.PI * 2)
        ctx.fillStyle = `rgb(${r}, ${g}, ${b})`
        ctx.fill()
        ctx.strokeStyle = COLORS.white
        ctx.lineWidth = 1
        ctx.stroke()
        
        // Distance label for recent targets
        if (age < 0.8) {
          ctx.font = '9px "JetBrains Mono", Consolas, monospace'
          ctx.fillStyle = COLORS.white
          ctx.textAlign = 'left'
          ctx.fillText(`${target.dist.toFixed(1)}m`, x + displaySize + 4, y + 3)
        }
      })
    }
    
    const drawBoat = (ctx, center) => {
      ctx.save()
      ctx.translate(center, center)
      
      // Boat body (circle)
      ctx.beginPath()
      ctx.arc(0, 0, 6, 0, Math.PI * 2)
      ctx.fillStyle = COLORS.green
      ctx.fill()
      ctx.strokeStyle = COLORS.white
      ctx.lineWidth = 1.5
      ctx.stroke()
      
      // Boat direction indicator (triangle pointing up/north)
      ctx.beginPath()
      ctx.moveTo(0, -12)
      ctx.lineTo(-5, -3)
      ctx.lineTo(5, -3)
      ctx.closePath()
      ctx.fillStyle = COLORS.cyan
      ctx.fill()
      ctx.strokeStyle = COLORS.white
      ctx.lineWidth = 1
      ctx.stroke()
      
      ctx.restore()
    }
    
    const drawLabels = (ctx, center, radius) => {
      // Heading display at bottom
      ctx.font = 'bold 12px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.cyan
      ctx.textAlign = 'center'
      ctx.fillText(`HDG ${heading.toFixed(0).padStart(3, '0')}°`, center, center + radius + 25)
    }
    
    draw()
    
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
  }, [heading, obstacles, maxRange, polarToXY])
  
  // Count active targets
  const activeTargets = Object.keys(targetsRef.current).length
  
  return (
    <div className="radar-panel">
      <div className="panel-header">
        <span className="panel-icon">◉</span>
        <h2>RADAR</h2>
      </div>
      
      <div className="radar-content">
        <canvas
          ref={canvasRef}
          width={280}
          height={280}
          className="radar-canvas"
        />
        
        <div className="radar-info">
          <div className="info-item">
            <span className="info-label">RANGE</span>
            <span className="info-value">{maxRange}m</span>
          </div>
          <div className="info-item">
            <span className="info-label">TARGETS</span>
            <span className="info-value danger">{obstacles.length}</span>
          </div>
        </div>
        
        <div className="range-controls">
          <button 
            className="range-btn"
            onClick={() => setMaxRange(Math.max(1, maxRange - 1))}
          >
            −
          </button>
          <span className="range-display">{maxRange}m</span>
          <button 
            className="range-btn"
            onClick={() => setMaxRange(Math.min(20, maxRange + 1))}
          >
            +
          </button>
        </div>
      </div>
    </div>
  )
}

export default Radar
