// HJ_261120 - Read-only driving view for the remote monitor (camrod_monitoring).
// The monitor embeds http://<robot>:8010/?view=driving-monitor in an iframe so it
// shows exactly the kiosk driving screen (3D scene + route card). Only GET
// /api/driving is used; no mission, stop, or return control is offered here.
import React from 'react';
import DrivingDisplay from './DrivingDisplay';
import { useDrivingMonitorSnapshot } from './useDrivingDisplay';
import './DrivingMonitor.css';

export default function DrivingMonitor() {
  const theme = new URLSearchParams(window.location.search).get('theme') === 'dark' ? 'dark' : 'light';
  const snapshot = useDrivingMonitorSnapshot();
  return (
    <div className="driving-monitor" data-ui="driving-monitor">
      <DrivingDisplay snapshot={snapshot} theme={theme} embedded readOnly />
    </div>
  );
}
