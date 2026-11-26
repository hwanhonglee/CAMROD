import React from 'react';
import DrivingDisplay from './DrivingDisplay';
import { useDrivingMonitorSnapshot } from './useDrivingDisplay';
import './DrivingMonitor.css';

// HH_261126 - Mount only the display, with no mission-authority callbacks.
export default function DrivingMonitor({ theme }) {
  const snapshot = useDrivingMonitorSnapshot();
  return (
    <div className="driving-monitor" data-ui="driving-monitor">
      <DrivingDisplay snapshot={snapshot} theme={theme} readOnly />
    </div>
  );
}
