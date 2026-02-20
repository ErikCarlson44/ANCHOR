import React, { useRef, useEffect, useState } from 'react'
import './Radar.css'

function Radar({ heading, obstacles }) {
  const canvasRef = useRef(null)
  const sweepAngleRef = useRef(0)
  const blipsRef = useRef([])
  const animationRef = useRef(null)
  
  const [maxRange, setMaxRange] = useState(4) // meters (0.5m to 8m for RD-03D)
  
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    
    const ctx = canvas.getContext('2d')
    const size = canvas.width
    const center = size / 2
    const radius = size / 2 - 30
    
    const draw = () => {
      // Clear canvas
      ctx.clearRect(0, 0, size, size)
      
      // Draw background
      drawBackground(ctx, center, radius)
      
      // Draw grid
      drawGrid(ctx, center, radius)
      
      // Draw blips (obstacles with fade)
      drawBlips(ctx, center, radius)
      
      // Draw sweep
      drawSweep(ctx, center, radius)
      
      // Draw boat indicator
      drawBoat(ctx, center)
      
      // Draw heading indicator
      drawHeading(ctx, center, radius, heading)
      
      // Draw labels
      drawLabels(ctx, center, radius)
      
      // Update sweep angle
      sweepAngleRef.current = (sweepAngleRef.current + 2) % 360
      
      // Check for obstacles in sweep area
      updateBlips()
      
      animationRef.current = requestAnimationFrame(draw)
    }
    
    const drawBackground = (ctx, center, radius) => {
      const gradient = ctx.createRadialGradient(center, center, 0, center, center, radius)
      gradient.addColorStop(0, '#0a2535')
      gradient.addColorStop(1, '#051520')
      
      ctx.beginPath()
      ctx.arc(center, center, radius, 0, Math.PI * 2)
      ctx.fillStyle = gradient
      ctx.fill()
      
      // Outer ring
      ctx.strokeStyle = '#00605050'
      ctx.lineWidth = 2
      ctx.stroke()
    }
    
    const drawGrid = (ctx, center, radius) => {
      ctx.strokeStyle = '#00505040'
      ctx.lineWidth = 1
      
      // Range circles
      for (let i = 1; i <= 4; i++) {
        const r = (radius / 4) * i
        ctx.beginPath()
        ctx.arc(center, center, r, 0, Math.PI * 2)
        ctx.stroke()
      }
      
      // Cardinal lines
      for (let angle = 0; angle < 360; angle += 45) {
        const rad = (angle - 90) * (Math.PI / 180)
        ctx.beginPath()
        ctx.moveTo(center, center)
        ctx.lineTo(
          center + radius * Math.cos(rad),
          center + radius * Math.sin(rad)
        )
        ctx.stroke()
      }
    }
    
    const drawSweep = (ctx, center, radius) => {
      const sweepRad = (sweepAngleRef.current - 90) * (Math.PI / 180)
      
      // Sweep trail gradient
      for (let i = 0; i < 40; i++) {
        const trailAngle = sweepAngleRef.current - i * 0.8
        const trailRad = (trailAngle - 90) * (Math.PI / 180)
        const alpha = 0.4 * (1 - i / 40)
        
        ctx.beginPath()
        ctx.moveTo(center, center)
        ctx.lineTo(
          center + radius * Math.cos(trailRad),
          center + radius * Math.sin(trailRad)
        )
        ctx.strokeStyle = `rgba(0, 255, 180, ${alpha})`
        ctx.lineWidth = 2
        ctx.stroke()
      }
      
      // Main sweep line
      ctx.beginPath()
      ctx.moveTo(center, center)
      ctx.lineTo(
        center + radius * Math.cos(sweepRad),
        center + radius * Math.sin(sweepRad)
      )
      ctx.strokeStyle = '#00ffb4'
      ctx.lineWidth = 3
      ctx.shadowColor = '#00ffb4'
      ctx.shadowBlur = 10
      ctx.stroke()
      ctx.shadowBlur = 0
    }
    
    const drawBlips = (ctx, center, radius) => {
      blipsRef.current = blipsRef.current.filter(blip => blip.age > 0)
      
      blipsRef.current.forEach(blip => {
        const x = center + blip.x * radius
        const y = center + blip.y * radius
        const size = 4 + blip.size * 3
        const alpha = blip.age
        
        // Glow
        const gradient = ctx.createRadialGradient(x, y, 0, x, y, size * 2)
        gradient.addColorStop(0, `rgba(255, 80, 100, ${alpha * 0.8})`)
        gradient.addColorStop(1, 'rgba(255, 80, 100, 0)')
        
        ctx.beginPath()
        ctx.arc(x, y, size * 2, 0, Math.PI * 2)
        ctx.fillStyle = gradient
        ctx.fill()
        
        // Core
        ctx.beginPath()
        ctx.arc(x, y, size, 0, Math.PI * 2)
        ctx.fillStyle = `rgba(255, 100, 120, ${alpha})`
        ctx.fill()
        
        // Fade
        blip.age -= 0.012
      })
    }
    
    const updateBlips = () => {
      // Always add current obstacles as blips (don't wait for sweep)
      obstacles.forEach(obs => {
        const angleRad = (obs.angle - 90) * (Math.PI / 180)
        const normDist = Math.min(obs.distance / maxRange, 1)
        
        // Only show if within range
        if (normDist > 0 && normDist <= 1) {
          const x = normDist * Math.cos(angleRad)
          const y = normDist * Math.sin(angleRad)
          
          // Check if blip already exists at this position
          const exists = blipsRef.current.some(
            b => Math.abs(b.x - x) < 0.08 && Math.abs(b.y - y) < 0.08 && b.age > 0.5
          )
          
          if (!exists) {
            blipsRef.current.push({
              x, y,
              size: obs.size || 1,
              age: 1
            })
          }
        }
      })
    }
    
    const drawBoat = (ctx, center) => {
      ctx.save()
      ctx.translate(center, center)
      
      // Boat triangle
      ctx.beginPath()
      ctx.moveTo(0, -10)
      ctx.lineTo(-6, 6)
      ctx.lineTo(6, 6)
      ctx.closePath()
      
      ctx.fillStyle = '#00ffd4'
      ctx.shadowColor = '#00ffd4'
      ctx.shadowBlur = 8
      ctx.fill()
      
      ctx.strokeStyle = '#00aa90'
      ctx.lineWidth = 1
      ctx.stroke()
      
      ctx.restore()
    }
    
    const drawHeading = (ctx, center, radius, heading) => {
      ctx.font = '600 12px "JetBrains Mono", monospace'
      ctx.fillStyle = '#00c8b4'
      ctx.textAlign = 'center'
      
      // N indicator
      ctx.fillText('N', center, center - radius + 18)
      
      // Heading value at bottom
      ctx.fillStyle = '#00ffd4'
      ctx.fillText(`${heading.toFixed(0).padStart(3, '0')}°`, center, center + radius - 8)
    }
    
    const drawLabels = (ctx, center, radius) => {
      ctx.font = '500 9px "JetBrains Mono", monospace'
      ctx.fillStyle = '#406860'
      ctx.textAlign = 'left'
      
      // Range labels
      for (let i = 1; i <= 4; i++) {
        const r = (radius / 4) * i
        const rangeValue = (maxRange / 4) * i
        ctx.fillText(`${rangeValue}m`, center + 4, center - r + 12)
      }
    }
    
    draw()
    
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
  }, [heading, obstacles, maxRange])
  
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
            <span className="info-label">OBJECTS</span>
            <span className="info-value danger">{obstacles.length}</span>
          </div>
        </div>
        
        <div className="range-controls">
          <button 
            className="range-btn"
            onClick={() => setMaxRange(Math.max(0.5, maxRange - 0.5))}
          >
            −
          </button>
          <span className="range-display">{maxRange}m</span>
          <button 
            className="range-btn"
            onClick={() => setMaxRange(Math.min(8, maxRange + 0.5))}
          >
            +
          </button>
        </div>
      </div>
    </div>
  )
}

export default Radar

