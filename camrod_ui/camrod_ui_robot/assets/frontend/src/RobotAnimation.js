import React from 'react';

// HH_260721 - Use chassis-neutral component naming for the Ranger operator UI.
export default function RobotAnimation() {
  return (
    <svg viewBox="0 0 100 100" fill="none" xmlns="http://www.w3.org/2000/svg"
      style={{ width: '100%', height: '100%' }}>
      {/* HH_260804 - Static range rings avoid permanent full-surface repaints
          on the Jetson operator display while preserving the location cue. */}
      <circle cx="50" cy="42" r="43" fill="none" stroke="#ef4444" strokeWidth="1.8" opacity="0.18" />
      <circle cx="50" cy="42" r="29" fill="none" stroke="#ef4444" strokeWidth="2" opacity="0.28" />
      {/* Pin body */}
      <path d="M50 14 C37 14 27 24 27 37 C27 53 50 80 50 80 C50 80 73 53 73 37 C73 24 63 14 50 14Z"
        fill="#ef4444"/>
      {/* Pin highlight */}
      <path d="M50 14 C37 14 27 24 27 37 C27 53 50 80 50 80 C50 80 73 53 73 37 C73 24 63 14 50 14Z"
        stroke="#c62828" strokeWidth="1.5"/>
      {/* Inner white circle */}
      <circle cx="50" cy="37" r="10" fill="white"/>
      {/* Red dot center */}
      <circle cx="50" cy="37" r="4" fill="#ef4444"/>
      {/* Shadow */}
      <ellipse cx="50" cy="84" rx="9" ry="4" fill="#ef4444" opacity="0.2"/>
    </svg>
  );
}
