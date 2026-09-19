import { useState, useEffect, useRef } from 'react';
import bridge from '../services/amrBridge';

export default function EventLog() {
  const [events, setEvents] = useState([]);
  const [filter, setFilter] = useState('all'); // 'all' | 'alerts' | 'nav'
  const [isExpanded, setIsExpanded] = useState(true);
  const logEndRef = useRef(null);

  useEffect(() => {
    // Initial welcome event
    setEvents([
      {
        id: 'init-0',
        time: new Date().toLocaleTimeString(),
        timestamp: Date.now(),
        level: 'info',
        message: 'Diagnostics Timeline initialized. Ready to record autonomous decisions.',
      },
    ]);

    const unsub = bridge.onEvent((evt) => {
      setEvents((prev) => [evt, ...prev.slice(0, 59)]); // Keep last 60 events
    });

    return unsub;
  }, []);

  const clearLogs = () => {
    setEvents([]);
  };

  const filteredEvents = events.filter((e) => {
    if (filter === 'alerts') return e.level === 'warn' || e.level === 'critical';
    if (filter === 'nav') return e.message.includes('Goal') || e.message.includes('Navigation') || e.message.includes('Mission');
    return true;
  });

  return (
    <div className="card event-log-card" style={{ flexShrink: 0, marginTop: 8 }}>
      <div className="card__title flex-row" style={{ justifyContent: 'space-between', alignItems: 'center' }}>
        <div className="flex-row" style={{ gap: 8, alignItems: 'center' }}>
          <span>📜 Autonomous Decisions &amp; Event Log</span>
          <span className="badge badge--cyan" style={{ fontSize: 10 }}>
            {filteredEvents.length} events
          </span>
        </div>

        <div className="flex-row" style={{ gap: 6 }}>
          <button
            className={`btn-filter ${filter === 'all' ? 'active' : ''}`}
            onClick={() => setFilter('all')}
          >
            All
          </button>
          <button
            className={`btn-filter ${filter === 'alerts' ? 'active' : ''}`}
            onClick={() => setFilter('alerts')}
          >
            Alerts
          </button>
          <button
            className={`btn-filter ${filter === 'nav' ? 'active' : ''}`}
            onClick={() => setFilter('nav')}
          >
            Missions
          </button>
          <button className="btn-filter" onClick={clearLogs} title="Clear event log">
            Clear
          </button>
          <button
            className="btn-filter"
            onClick={() => setIsExpanded(!isExpanded)}
            title={isExpanded ? 'Collapse' : 'Expand'}
          >
            {isExpanded ? '▲' : '▼'}
          </button>
        </div>
      </div>

      {isExpanded && (
        <div className="event-log-list mt-sm">
          {filteredEvents.length === 0 ? (
            <div className="text-dim text-xs" style={{ padding: '8px 0', textAlign: 'center' }}>
              No events recorded for filter.
            </div>
          ) : (
            filteredEvents.map((evt) => {
              const badgeClass =
                evt.level === 'critical' ? 'badge--red' :
                evt.level === 'warn'     ? 'badge--yellow' :
                evt.level === 'success'  ? 'badge--green' : 'badge--cyan';

              return (
                <div key={evt.id} className="event-log-item">
                  <span className="event-time text-mono">{evt.time}</span>
                  <span className={`badge ${badgeClass} text-xs`}>
                    {evt.level.toUpperCase()}
                  </span>
                  <span className="event-message">{evt.message}</span>
                </div>
              );
            })
          )}
          <div ref={logEndRef} />
        </div>
      )}
    </div>
  );
}
