import React, { useRef, useEffect } from 'react'
import './Radar.css'

function Radar({ heading, obstacles }) {
  const canvasRef = useRef(null)
  const animationRef = useRef(null)
  const sectorDataRef = useRef({ left: null, center: null, right: null })
  const sweepAngleRef = useRef(0)
  const sweepDirectionRef = useRef(1)
  
  // Servo range: 0° to 180° (90° is forward/center)
  const SERVO_MIN = 0
  const SERVO_MAX = 180
  
  // Distance range: 1-6 meters
  const DIST_MIN = 1
  const DIST_MAX = 6
  
  // Sector boundaries (servo angles)
  // Right sector: 0-60°, Center sector: 60-120°, Left sector: 120-180°
  const SECTORS = [
    { name: 'right', start: 0, end: 60 },
    { name: 'center', start: 60, end: 120 },
    { name: 'left', start: 120, end: 180 }
  ]
  
  // Process obstacles into sector data (closest detection per sector)
  useEffect(() => {
    const newSectorData = { left: null, center: null, right: null }
    
    if (obstacles && obstacles.length > 0) {
      obstacles.forEach(obs => {
        // obs.angle is relative: negative = left, positive = right, 0 = forward
        const servoAngle = 90 - obs.angle // Convert to servo angle (90 = forward)
        
        // Determine which sector this falls into
        let sectorName = null
        if (servoAngle >= 0 && servoAngle < 60) sectorName = 'right'
        else if (servoAngle >= 60 && servoAngle < 120) sectorName = 'center'
        else if (servoAngle >= 120 && servoAngle <= 180) sectorName = 'left'
        
        if (sectorName) {
          // Round to nearest meter
          const dist = Math.round(Math.min(DIST_MAX, Math.max(DIST_MIN, obs.distance)))
          
          // Keep closest detection in each sector
          if (newSectorData[sectorName] === null || dist < newSectorData[sectorName]) {
            newSectorData[sectorName] = dist
          }
        }
      })
    }
    
    sectorDataRef.current = newSectorData
  }, [obstacles])
  
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    
    const ctx = canvas.getContext('2d')
    const width = canvas.width
    const height = canvas.height
    
    const COLORS = {
      bg: '#0a1a15',
      grid: '#1a3a2a',
      green: '#00ff00',
      brightGreen: '#00ff64',
      darkGreen: '#004400',
      cyan: '#00ffff',
      yellow: '#ffff00',
      orange: '#ff8800',
      red: '#ff4444',
      white: '#ffffff'
    }
    
    // Get color based on distance
    const getDistanceColor = (dist, alpha = 0.6) => {
      if (dist <= 2) return `rgba(255, 68, 68, ${alpha})`   // red
      if (dist <= 3) return `rgba(255, 136, 0, ${alpha})`   // orange
      if (dist <= 4) return `rgba(255, 255, 0, ${alpha})`   // yellow
      return `rgba(0, 255, 0, ${alpha})`                     // green
    }
    
    const draw = () => {
      const sectorData = sectorDataRef.current
      
      // Clear canvas
      ctx.fillStyle = COLORS.bg
      ctx.fillRect(0, 0, width, height)
      
      // Center point at bottom center
      const centerX = width / 2
      const centerY = height - 35
      const maxRadius = height - 60
      
      // Convert servo angle to canvas angle
      // Servo 90° = forward = straight UP = -90° in canvas
      // Servo 0° = right = 0° in canvas
      // Servo 180° = left = -180° in canvas
      const servoToCanvas = (servoAngle) => (-servoAngle) * Math.PI / 180
      
      // Arc from 180° (left) to 0° (right) - full semicircle
      const startRad = servoToCanvas(SERVO_MAX) // -180° = left side
      const endRad = servoToCanvas(SERVO_MIN)   // 0° = right side
      
      // Draw arc background
      ctx.beginPath()
      ctx.moveTo(centerX, centerY)
      ctx.arc(centerX, centerY, maxRadius, startRad, endRad)
      ctx.closePath()
      ctx.fillStyle = '#061510'
      ctx.fill()
      ctx.strokeStyle = COLORS.darkGreen
      ctx.lineWidth = 2
      ctx.stroke()
      
      // Draw the 3 shaded sectors if obstacles detected
      SECTORS.forEach(sector => {
        const dist = sectorData[sector.name]
        
        if (dist !== null && dist !== undefined) {
          // Calculate radius based on distance
          const sectorRadius = (dist / DIST_MAX) * maxRadius
          
          // Convert sector angles to canvas angles
          const sectorStartRad = servoToCanvas(sector.end)   // Reversed because canvas goes clockwise
          const sectorEndRad = servoToCanvas(sector.start)
          
          // Draw filled sector (pie wedge)
          ctx.beginPath()
          ctx.moveTo(centerX, centerY)
          ctx.arc(centerX, centerY, sectorRadius, sectorStartRad, sectorEndRad)
          ctx.closePath()
          
          // Fill with distance-based color
          ctx.fillStyle = getDistanceColor(dist, 0.5)
          ctx.fill()
          
          // Brighter border for the sector
          ctx.strokeStyle = getDistanceColor(dist, 1.0)
          ctx.lineWidth = 2
          ctx.stroke()
          
          // Draw glow effect for close objects
          if (dist <= 3) {
            ctx.shadowColor = getDistanceColor(dist, 1.0)
            ctx.shadowBlur = 15
            ctx.stroke()
            ctx.shadowBlur = 0
          }
          
          // Distance label in the middle of the sector
          const midAngle = servoToCanvas((sector.start + sector.end) / 2)
          const labelRadius = sectorRadius * 0.6
          const labelX = centerX + labelRadius * Math.cos(midAngle)
          const labelY = centerY + labelRadius * Math.sin(midAngle)
          
          ctx.font = 'bold 10px "JetBrains Mono", Consolas, monospace'
          ctx.fillStyle = COLORS.white
          ctx.textAlign = 'center'
          ctx.textBaseline = 'middle'
          ctx.fillText(`${dist}m`, labelX, labelY)
        }
      })
      
      // Draw distance arcs (1m, 2m, 3m, 4m, 5m, 6m)
      for (let d = DIST_MIN; d <= DIST_MAX; d++) {
        const r = (d / DIST_MAX) * maxRadius
        ctx.beginPath()
        ctx.arc(centerX, centerY, r, startRad, endRad)
        ctx.strokeStyle = COLORS.grid
        ctx.lineWidth = 1
        ctx.stroke()
        
        // Distance label on the right side
        const labelAngle = servoToCanvas(15)
        const labelX = centerX + r * Math.cos(labelAngle)
        const labelY = centerY + r * Math.sin(labelAngle)
        ctx.font = '8px "JetBrains Mono", Consolas, monospace'
        ctx.fillStyle = COLORS.darkGreen
        ctx.textAlign = 'left'
        ctx.fillText(`${d}m`, labelX + 2, labelY)
      }
      
      // Draw sector boundary lines (0°, 60°, 120°, 180°)
      const sectorBoundaries = [0, 60, 120, 180]
      sectorBoundaries.forEach(angle => {
        const canvasAngle = servoToCanvas(angle)
        ctx.beginPath()
        ctx.moveTo(centerX, centerY)
        ctx.lineTo(
          centerX + maxRadius * Math.cos(canvasAngle),
          centerY + maxRadius * Math.sin(canvasAngle)
        )
        ctx.strokeStyle = angle === 60 || angle === 120 ? COLORS.green : COLORS.grid
        ctx.lineWidth = angle === 60 || angle === 120 ? 1.5 : 1
        ctx.stroke()
        
        // Angle label
        const labelR = maxRadius + 12
        const labelX = centerX + labelR * Math.cos(canvasAngle)
        const labelY = centerY + labelR * Math.sin(canvasAngle)
        ctx.font = '9px "JetBrains Mono", Consolas, monospace'
        ctx.fillStyle = COLORS.green
        ctx.textAlign = 'center'
        ctx.textBaseline = 'middle'
        ctx.fillText(`${angle}°`, labelX, labelY)
      })
      
      // Draw forward indicator at 90°
      const fwdAngle = servoToCanvas(90)
      ctx.beginPath()
      ctx.moveTo(centerX, centerY)
      ctx.lineTo(
        centerX + maxRadius * Math.cos(fwdAngle),
        centerY + maxRadius * Math.sin(fwdAngle)
      )
      ctx.strokeStyle = COLORS.cyan
      ctx.lineWidth = 1
      ctx.setLineDash([4, 4])
      ctx.stroke()
      ctx.setLineDash([])
      
      // FWD label
      ctx.font = 'bold 9px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.cyan
      ctx.fillText('FWD', centerX, centerY - maxRadius - 10)
      
      // Draw sweep animation
      const sweepAngle = sweepAngleRef.current
      const sweepRad = servoToCanvas(sweepAngle)
      
      // Sweep trail (fading lines behind the sweep)
      for (let i = 1; i < 12; i++) {
        const trailAngle = sweepAngle - (i * 3 * sweepDirectionRef.current)
        if (trailAngle < SERVO_MIN || trailAngle > SERVO_MAX) continue
        
        const trailRad = servoToCanvas(trailAngle)
        const intensity = Math.max(0, 60 - i * 5)
        
        ctx.beginPath()
        ctx.moveTo(centerX, centerY)
        ctx.lineTo(
          centerX + maxRadius * Math.cos(trailRad),
          centerY + maxRadius * Math.sin(trailRad)
        )
        ctx.strokeStyle = `rgb(0, ${intensity}, 0)`
        ctx.lineWidth = Math.max(1, 2 - i * 0.1)
        ctx.stroke()
      }
      
      // Main sweep line
      ctx.beginPath()
      ctx.moveTo(centerX, centerY)
      ctx.lineTo(
        centerX + maxRadius * Math.cos(sweepRad),
        centerY + maxRadius * Math.sin(sweepRad)
      )
      ctx.strokeStyle = COLORS.brightGreen
      ctx.lineWidth = 2
      ctx.shadowColor = COLORS.brightGreen
      ctx.shadowBlur = 8
      ctx.stroke()
      ctx.shadowBlur = 0
      
      // Update sweep angle
      sweepAngleRef.current += 3 * sweepDirectionRef.current
      if (sweepAngleRef.current >= SERVO_MAX) {
        sweepAngleRef.current = SERVO_MAX
        sweepDirectionRef.current = -1
      } else if (sweepAngleRef.current <= SERVO_MIN) {
        sweepAngleRef.current = SERVO_MIN
        sweepDirectionRef.current = 1
      }
      
      // Draw center point (boat position)
      ctx.beginPath()
      ctx.arc(centerX, centerY, 5, 0, Math.PI * 2)
      ctx.fillStyle = COLORS.green
      ctx.fill()
      ctx.strokeStyle = COLORS.white
      ctx.lineWidth = 1.5
      ctx.stroke()
      
      // Draw boat direction indicator (pointing up)
      ctx.beginPath()
      ctx.moveTo(centerX, centerY - 12)
      ctx.lineTo(centerX - 4, centerY - 3)
      ctx.lineTo(centerX + 4, centerY - 3)
      ctx.closePath()
      ctx.fillStyle = COLORS.cyan
      ctx.fill()
      
      // Title
      ctx.font = 'bold 10px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.cyan
      ctx.textAlign = 'left'
      ctx.fillText('RADAR', 8, 15)
      
      // Heading display
      ctx.textAlign = 'right'
      ctx.fillText(`HDG ${heading.toFixed(0).padStart(3, '0')}°`, width - 8, 15)
      
      // Sector labels
      ctx.font = '8px "JetBrains Mono", Consolas, monospace'
      ctx.fillStyle = COLORS.darkGreen
      ctx.textAlign = 'left'
      ctx.fillText('R', width - 25, height - 15)
      ctx.textAlign = 'center'
      ctx.fillText('C', centerX, 28)
      ctx.textAlign = 'right'
      ctx.fillText('L', 25, height - 15)
      
      animationRef.current = requestAnimationFrame(draw)
    }
    
    draw()
    
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
  }, [heading])
  
  // Get sector data for info display
  const sectorData = sectorDataRef.current
  const activeCount = [sectorData.left, sectorData.center, sectorData.right].filter(d => d !== null).length
  const allDistances = [sectorData.left, sectorData.center, sectorData.right].filter(d => d !== null)
  const closestDist = allDistances.length > 0 ? Math.min(...allDistances) : DIST_MAX
  
  return (
    <div className="radar-panel">
      <div className="panel-header">
        <span className="panel-icon">◉</span>
        <h2>RADAR STATUS</h2>
      </div>
      
      <div className="radar-content">
        <canvas
          ref={canvasRef}
          width={280}
          height={180}
          className="radar-canvas"
        />
        
        <div className="radar-info">
          <div className="info-item">
            <span className="info-label">LEFT</span>
            <span className={`info-value ${sectorData.left && sectorData.left <= 3 ? 'danger' : ''}`}>
              {sectorData.left ? `${sectorData.left}m` : '--'}
            </span>
          </div>
          <div className="info-item">
            <span className="info-label">CENTER</span>
            <span className={`info-value ${sectorData.center && sectorData.center <= 3 ? 'danger' : ''}`}>
              {sectorData.center ? `${sectorData.center}m` : '--'}
            </span>
          </div>
          <div className="info-item">
            <span className="info-label">RIGHT</span>
            <span className={`info-value ${sectorData.right && sectorData.right <= 3 ? 'danger' : ''}`}>
              {sectorData.right ? `${sectorData.right}m` : '--'}
            </span>
          </div>
          <div className="info-item">
            <span className="info-label">CLOSEST</span>
            <span className={`info-value ${closestDist <= 3 ? 'danger' : ''}`}>
              {activeCount > 0 ? `${closestDist}m` : '--'}
            </span>
          </div>
        </div>
      </div>
    </div>
  )
}

export default Radar
