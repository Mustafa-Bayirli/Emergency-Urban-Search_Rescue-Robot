import React from 'react';

function Analytics() {
  return (
    <div style={{ display: 'flex' }}>
      <div style={{ width: '66.66%' }}>
        <iframe className="flip" src="http://127.0.0.1:5000" controls style={{ width: '100%', height: '75vh' }} />
      </div>
      <div style={{ width: '33.33%' }}>
        {/* pointcloud map */}
      </div>
    </div>
  );
}

export default Analytics;
