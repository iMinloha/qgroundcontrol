#pragma once

#include "BaseClasses/VehicleTestManualConnect.h"

/// Tests for the _HASH_CHECK parameter cache optimization.
/// See test matrix in ParameterManager for all scenarios covered.
class HashCheckTest : public VehicleTestManualConnect
{
    Q_OBJECT

private slots:
    void cleanup() override;

    // Scenario 1: First connect, no cache file — falls back to PARAM_REQUEST_LIST
    void _firstConnectNoCache();

    // Scenario 2: Reconnect with valid cache — loads from cache, no PARAM_REQUEST_LIST
    void _reconnectCacheHit();

    // Scenario 3: Reconnect after param change — cache miss, falls back to PARAM_REQUEST_LIST
    void _reconnectCacheMiss();

    // Scenario 4: Hash check times out, cache valid — falls back to PARAM_REQUEST_LIST,
    //             then _HASH_CHECK in stream triggers cache load
    void _hashCheckTimeoutCacheHit();

    // Scenario 5: Hash check times out, no cache — falls back to PARAM_REQUEST_LIST,
    //             stream continues normally
    void _hashCheckTimeoutNoCache();

    // Scenario 6: Hash check times out, cache stale — falls back to PARAM_REQUEST_LIST,
    //             then _HASH_CHECK in stream → cache miss → stream continues
    void _hashCheckTimeoutCacheStale();

    // Scenario 7: Both timers exhaust — no response at all
    void _bothTimersExhaust();

    // Scenario 8: Cache deleted between connects — hash check response arrives but no cache file
    void _cacheDeletedBetweenConnects();

    // Scenario 9: Manual refresh after initial load — goes straight to PARAM_REQUEST_LIST
    void _manualRefreshAfterLoad();

    // Scenario 10: ArduPilot vehicle — skips hash check entirely
    void _arduPilotSkipsHashCheck();

    // Scenario 11: High latency link — signals ready immediately, no hash check
    void _highLatencyLink();

    // Scenario 12: Log replay — same skip path as high latency
    void _logReplay();

private:
    void _deleteCacheFiles();
    void _connectAndWaitForParams();
    void _disconnectAndSettle();
    MockLink *_startPX4MockLinkNoIncrement(MockConfiguration::FailureMode_t failureMode = MockConfiguration::FailNone);
    MockLink *_startPX4MockLinkHighLatency();
};
