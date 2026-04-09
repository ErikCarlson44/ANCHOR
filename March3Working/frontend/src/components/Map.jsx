import React, { useEffect, useRef, useState, useMemo } from 'react'
import { MapContainer, TileLayer, Marker, Polyline, useMap } from 'react-leaflet'
import L from 'leaflet'
import './Map.css'

// Process obstacles into 3 sectors (matches Radar.jsx logic)
const processObstaclesIntoSectors = (obstacles) => {
  const DIST_MIN = 1
  const DIST_MAX = 6
  const sectorData = { left: null, center: null, right: null }
  
  if (obstacles && obstacles.length > 0) {
    obstacles.forEach(obs => {
      const servoAngle = 90 - obs.angle
      
      let sectorName = null
      if (servoAngle >= 0 && servoAngle < 60) sectorName = 'right'
      else if (servoAngle >= 60 && servoAngle < 120) sectorName = 'center'
      else if (servoAngle >= 120 && servoAngle <= 180) sectorName = 'left'
      
      if (sectorName) {
        const dist = Math.round(Math.min(DIST_MAX, Math.max(DIST_MIN, obs.distance)))
        if (sectorData[sectorName] === null || dist < sectorData[sectorName]) {
          sectorData[sectorName] = dist
        }
      }
    })
  }
  
  return sectorData
}

// Get color based on distance
const getDistanceColor = (dist, alpha = 0.4) => {
  if (dist <= 2) return `rgba(255, 68, 68, ${alpha})`
  if (dist <= 3) return `rgba(255, 136, 0, ${alpha})`
  if (dist <= 4) return `rgba(255, 255, 0, ${alpha})`
  return `rgba(0, 255, 0, ${alpha})`
}

// Custom boat icon
const createBoatIcon = (heading) => {
  return L.divIcon({
    className: 'boat-marker',
    html: `
      <div class="boat-icon" style="transform: rotate(${heading}deg)">
        <svg viewBox="0 0 40 50" width="40" height="50">
          <defs>
            <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
              <feGaussianBlur stdDeviation="2" result="coloredBlur"/>
              <feMerge>
                <feMergeNode in="coloredBlur"/>
                <feMergeNode in="SourceGraphic"/>
              </feMerge>
            </filter>
            <linearGradient id="boatGrad" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" style="stop-color:#00ffcc;stop-opacity:1" />
              <stop offset="100%" style="stop-color:#00aa88;stop-opacity:0.8" />
            </linearGradient>
          </defs>
          <polygon 
            points="20,2 35,45 20,38 5,45" 
            fill="url(#boatGrad)" 
            stroke="#00ffcc" 
            stroke-width="1.5"
            filter="url(#glow)"
          />
          <polygon 
            points="20,10 28,38 20,32 12,38" 
            fill="rgba(0,20,30,0.5)"
          />
        </svg>
      </div>
    `,
    iconSize: [40, 50],
    iconAnchor: [20, 25]
  })
}

// Component to update map view
function MapUpdater({ position, follow }) {
  const map = useMap()
  
  useEffect(() => {
    if (follow) {
      map.setView(position, map.getZoom(), { animate: true })
    }
  }, [position, follow, map])
  
  return null
}

// Component to draw radar sectors on the map
function RadarOverlay({ position, heading, sectorData }) {
  const map = useMap()
  const canvasRef = useRef(null)
  
  useEffect(() => {
    if (!map) return
    
    // Create or get the canvas overlay pane
    let pane = map.getPane('radarPane')
    if (!pane) {
      pane = map.createPane('radarPane')
      pane.style.zIndex = 450
      pane.style.pointerEvents = 'none'
    }
    
    // Create canvas if it doesn't exist
    if (!canvasRef.current) {
      canvasRef.current = document.createElement('canvas')
      canvasRef.current.style.position = 'absolute'
      canvasRef.current.style.pointerEvents = 'none'
      pane.appendChild(canvasRef.current)
    }
    
    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    
    const drawRadar = () => {
      const size = map.getSize()
      canvas.width = size.x
      canvas.height = size.y
      
      // Position canvas
      const topLeft = map.containerPointToLayerPoint([0, 0])
      L.DomUtil.setPosition(canvas, topLeft)
      
      ctx.clearRect(0, 0, canvas.width, canvas.height)
      
      // Get boat position in pixels
      const boatPoint = map.latLngToContainerPoint(position)
      const centerX = boatPoint.x
      const centerY = boatPoint.y
      
      // Calculate pixel radius based on zoom (6 meters max)
      const metersPerPixel = 40075016.686 * Math.cos(position[0] * Math.PI / 180) / Math.pow(2, map.getZoom() + 8)
      const maxRadiusMeters = 6
      const maxRadiusPixels = maxRadiusMeters / metersPerPixel
      
      // Heading offset (convert boat heading to canvas angle)
      const headingRad = (heading - 90) * Math.PI / 180
      
      // Draw each sector (angles relative to forward direction)
      const sectors = [
        { name: 'right', startAngle: 30, endAngle: 90 },     // 0-60° servo = right of boat
        { name: 'center', startAngle: -30, endAngle: 30 },   // 60-120° servo = forward
        { name: 'left', startAngle: -90, endAngle: -30 }     // 120-180° servo = left of boat
      ]
      
      sectors.forEach(sector => {
        const dist = sectorData[sector.name]
        if (dist === null || dist === undefined) return
        
        const sectorRadius = (dist / 6) * maxRadiusPixels
        
        // Adjust angles for boat heading
        const startRad = (sector.startAngle * Math.PI / 180) + headingRad
        const endRad = (sector.endAngle * Math.PI / 180) + headingRad
        
        // Draw filled sector
        ctx.beginPath()
        ctx.moveTo(centerX, centerY)
        ctx.arc(centerX, centerY, sectorRadius, startRad, endRad)
        ctx.closePath()
        
        ctx.fillStyle = getDistanceColor(dist, 0.35)
        ctx.fill()
        
        ctx.strokeStyle = getDistanceColor(dist, 0.8)
        ctx.lineWidth = 2
        ctx.stroke()
        
        // Draw distance label
        const midAngle = (startRad + endRad) / 2
        const labelRadius = sectorRadius * 0.65
        const labelX = centerX + labelRadius * Math.cos(midAngle)
        const labelY = centerY + labelRadius * Math.sin(midAngle)
        
        ctx.font = 'bold 11px "JetBrains Mono", Consolas, monospace'
        ctx.fillStyle = '#ffffff'
        ctx.textAlign = 'center'
        ctx.textBaseline = 'middle'
        
        // Draw text with shadow for visibility
        ctx.shadowColor = 'rgba(0, 0, 0, 0.8)'
        ctx.shadowBlur = 3
        ctx.fillText(`${dist}m`, labelX, labelY)
        ctx.shadowBlur = 0
      })
      
      // Draw outer range circle (6m boundary)
      ctx.beginPath()
      ctx.arc(centerX, centerY, maxRadiusPixels, 0, Math.PI * 2)
      ctx.strokeStyle = 'rgba(0, 255, 100, 0.2)'
      ctx.lineWidth = 1
      ctx.setLineDash([5, 5])
      ctx.stroke()
      ctx.setLineDash([])
    }
    
    drawRadar()
    
    // Redraw on map events
    map.on('move', drawRadar)
    map.on('zoom', drawRadar)
    map.on('resize', drawRadar)
    
    return () => {
      map.off('move', drawRadar)
      map.off('zoom', drawRadar)
      map.off('resize', drawRadar)
    }
  }, [map, position, heading, sectorData])
  
  return null
}

function Map({ position, heading, obstacles, trail }) {
  const [followBoat, setFollowBoat] = useState(true)
  const boatIcon = createBoatIcon(heading)
  
  // Process obstacles into sectors (memoized)
  const sectorData = useMemo(() => processObstaclesIntoSectors(obstacles), [obstacles])
  
  return (
    <div className="map-wrapper">
      <div className="map-header">
        <div className="map-title">
          <span className="map-icon">◎</span>
          <span>NAVIGATION MAP</span>
        </div>
        <div className="map-controls">
          <button 
            className={`map-btn ${followBoat ? 'active' : ''}`}
            onClick={() => setFollowBoat(!followBoat)}
          >
            {followBoat ? '◉ TRACKING' : '○ FREE'}
          </button>
        </div>
      </div>
      
      <MapContainer
        center={position}
        zoom={21}
        className="leaflet-map"
        zoomControl={true}
        maxZoom={22}
      >
        <TileLayer
          attribution='&copy; <a href="https://carto.com/">CARTO</a>'
          url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png"
          maxZoom={22}
          maxNativeZoom={19}
        />
        
        <MapUpdater position={position} follow={followBoat} />
        
        {/* Radar sector overlay */}
        <RadarOverlay position={position} heading={heading} sectorData={sectorData} />
        
        {/* Trail */}
        {trail.length > 1 && (
          <Polyline
            positions={trail}
            pathOptions={{
              color: '#00ffcc',
              weight: 2,
              opacity: 0.6,
              dashArray: '5, 10'
            }}
          />
        )}
        
        {/* Boat marker */}
        <Marker position={position} icon={boatIcon} />
      </MapContainer>
      
      <div className="map-overlay">
        <div className="coord-display">
          <span className="coord-label">LAT</span>
          <span className="coord-value">{position[0].toFixed(6)}</span>
        </div>
        <div className="coord-display">
          <span className="coord-label">LON</span>
          <span className="coord-value">{position[1].toFixed(6)}</span>
        </div>
        <div className="coord-display">
          <span className="coord-label">HDG</span>
          <span className="coord-value">{heading.toFixed(0)}°</span>
        </div>
      </div>
    </div>
  )
}

export default Map
