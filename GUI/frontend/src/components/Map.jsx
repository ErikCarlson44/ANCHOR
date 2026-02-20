import React, { useEffect, useRef, useState } from 'react'
import { MapContainer, TileLayer, Marker, Polyline, CircleMarker, useMap } from 'react-leaflet'
import L from 'leaflet'
import './Map.css'

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

function Map({ position, heading, obstacles, trail }) {
  const [followBoat, setFollowBoat] = useState(true)
  const boatIcon = createBoatIcon(heading)
  
  // Convert obstacles to map markers
  const obstacleMarkers = obstacles.map((obs, index) => {
    const angleRad = (heading + obs.angle) * (Math.PI / 180)
    const distDeg = obs.distance / 111000 // Rough conversion
    const lat = position[0] + distDeg * Math.cos(angleRad)
    const lon = position[1] + distDeg * Math.sin(angleRad) / Math.cos(position[0] * Math.PI / 180)
    return { lat, lon, size: obs.size, key: index }
  })
  
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
        zoom={17}
        className="leaflet-map"
        zoomControl={true}
      >
        <TileLayer
          attribution='&copy; <a href="https://carto.com/">CARTO</a>'
          url="https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png"
        />
        
        <MapUpdater position={position} follow={followBoat} />
        
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
        
        {/* Obstacle markers */}
        {obstacleMarkers.map((obs) => (
          <CircleMarker
            key={obs.key}
            center={[obs.lat, obs.lon]}
            radius={6 + obs.size * 2}
            pathOptions={{
              color: '#ff4466',
              fillColor: '#ff4466',
              fillOpacity: 0.6,
              weight: 2
            }}
          />
        ))}
        
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

