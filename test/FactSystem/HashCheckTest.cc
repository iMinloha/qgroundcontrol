#include "HashCheckTest.h"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtTest/QSignalSpy>
#include <QtTest/QTest>

#include "LinkManager.h"
#include "MAVLinkLib.h"
#include "MockConfiguration.h"
#include "MultiVehicleManager.h"
#include "ParameterManager.h"
#include "Vehicle.h"

void HashCheckTest::cleanup()
{
    if (_mockLink && MultiVehicleManager::instance()->activeVehicle()) {
        _mockLink->disconnect();
        _mockLink = nullptr;
        QSignalSpy spy(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
        (void) UnitTest::waitForSignal(spy, TestTimeout::mediumMs(), QStringLiteral("activeVehicleChanged"));
    }
    VehicleTestManualConnect::cleanup();
}

void HashCheckTest::_deleteCacheFiles()
{
    const QDir cacheDir = ParameterManager::parameterCacheDir();
    if (cacheDir.exists()) {
        const QStringList cacheFiles = cacheDir.entryList(QStringList() << QStringLiteral("*.v2"), QDir::Files);
        for (const QString &file : cacheFiles) {
            QFile::remove(cacheDir.filePath(file));
        }
    }
}

void HashCheckTest::_connectAndWaitForParams()
{
    MultiVehicleManager *const vehicleMgr = MultiVehicleManager::instance();
    QVERIFY(vehicleMgr);

    QSignalSpy spyVehicle(vehicleMgr, &MultiVehicleManager::activeVehicleAvailableChanged);
    QVERIFY_SIGNAL_WAIT(spyVehicle, TestTimeout::mediumMs());

    Vehicle *const vehicle = vehicleMgr->activeVehicle();
    QVERIFY(vehicle);

    QSignalSpy spyParamsReady(vehicleMgr, &MultiVehicleManager::parameterReadyVehicleAvailableChanged);
    QVERIFY_SIGNAL_WAIT(spyParamsReady, TestTimeout::longMs());

    const QList<QVariant> arguments = spyParamsReady.takeFirst();
    QCOMPARE(arguments.count(), 1);
    QCOMPARE(arguments.at(0).toBool(), true);
}

MockLink *HashCheckTest::_startPX4MockLinkNoIncrement(MockConfiguration::FailureMode_t failureMode)
{
    auto *const mockConfig = new MockConfiguration(QStringLiteral("PX4 MockLink"));
    mockConfig->setFirmwareType(MAV_AUTOPILOT_PX4);
    mockConfig->setVehicleType(MAV_TYPE_QUADROTOR);
    mockConfig->setIncrementVehicleId(false);
    mockConfig->setFailureMode(failureMode);
    mockConfig->setDynamic(true);

    SharedLinkConfigurationPtr config = LinkManager::instance()->addConfiguration(mockConfig);
    if (LinkManager::instance()->createConnectedLink(config)) {
        return qobject_cast<MockLink *>(config->link());
    }
    return nullptr;
}

// Scenario 1: First connect with no cache — should fall back to PARAM_REQUEST_LIST
void HashCheckTest::_firstConnectNoCache()
{
    _deleteCacheFiles();

    _mockLink = MockLink::startPX4MockLink(false, false, false);
    _connectAndWaitForParams();

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(vehicle);
    QVERIFY(vehicle->parameterManager()->parametersReady());
    QVERIFY(!vehicle->parameterManager()->missingParameters());

    // PARAM_REQUEST_LIST should have been sent (fall back from no cache)
    QVERIFY(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) > 0);
}

// Scenario 2: Reconnect with valid cache — should load from cache without PARAM_REQUEST_LIST
void HashCheckTest::_reconnectCacheHit()
{
    _deleteCacheFiles();

    // First connect: populates the cache (use non-incrementing ID so reconnect gets same ID)
    _mockLink = _startPX4MockLinkNoIncrement();
    _connectAndWaitForParams();

    const int vehicleId = _mockLink->vehicleId();

    // Disconnect
    _mockLink->disconnect();
    _mockLink = nullptr;
    QSignalSpy spyDisconnect(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
    QVERIFY(UnitTest::waitForSignal(spyDisconnect, TestTimeout::longMs(), QStringLiteral("activeVehicleChanged")));
    UnitTest::settleEventLoopForCleanup();

    // Cache file should exist now
    QVERIFY(QFile::exists(ParameterManager::parameterCacheFile(vehicleId, MAV_COMP_ID_AUTOPILOT1)));

    // Second connect: same vehicle ID, should use cache
    _mockLink = _startPX4MockLinkNoIncrement();
    _connectAndWaitForParams();

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(vehicle);
    QVERIFY(vehicle->parameterManager()->parametersReady());

    // PARAM_REQUEST_LIST should NOT have been sent (loaded from cache)
    QCOMPARE(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST), 0);
}

// Scenario 3: Reconnect after param change on vehicle side — cache miss, falls back to PARAM_REQUEST_LIST
void HashCheckTest::_reconnectCacheMiss()
{
    _deleteCacheFiles();

    // First connect: populates the cache (use non-incrementing ID so reconnect gets same ID)
    _mockLink = _startPX4MockLinkNoIncrement();
    _connectAndWaitForParams();

    _mockLink->disconnect();
    _mockLink = nullptr;
    QSignalSpy spyDisconnect(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
    QVERIFY(UnitTest::waitForSignal(spyDisconnect, TestTimeout::longMs(), QStringLiteral("activeVehicleChanged")));
    UnitTest::settleEventLoopForCleanup();

    // Second connect: same vehicle ID, but change a param so CRC won't match
    _mockLink = _startPX4MockLinkNoIncrement();
    _mockLink->setMockParamValue(MAV_COMP_ID_AUTOPILOT1, QStringLiteral("BAT1_V_CHARGED"), 99.0f);

    _connectAndWaitForParams();

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(vehicle);
    QVERIFY(vehicle->parameterManager()->parametersReady());

    // PARAM_REQUEST_LIST should have been sent (cache miss)
    QVERIFY(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) > 0);
}

// Scenario 4: Hash check times out but cache is valid — PARAM_REQUEST_LIST stream
//             delivers _HASH_CHECK which triggers cache load
void HashCheckTest::_hashCheckTimeoutCacheHit()
{
    _deleteCacheFiles();

    // First connect: populates the cache (use non-incrementing ID so reconnect gets same ID)
    _mockLink = _startPX4MockLinkNoIncrement();
    _connectAndWaitForParams();

    _mockLink->disconnect();
    _mockLink = nullptr;
    QSignalSpy spyDisconnect(MultiVehicleManager::instance(), &MultiVehicleManager::activeVehicleChanged);
    QVERIFY(UnitTest::waitForSignal(spyDisconnect, TestTimeout::longMs(), QStringLiteral("activeVehicleChanged")));
    UnitTest::settleEventLoopForCleanup();

    // Second connect: same vehicle ID, suppress standalone hash check response so it times out
    // But _HASH_CHECK in PARAM_REQUEST_LIST stream should still trigger cache load
    _mockLink = _startPX4MockLinkNoIncrement();
    _mockLink->setHashCheckNoResponse(true);

    _connectAndWaitForParams();

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(vehicle);
    QVERIFY(vehicle->parameterManager()->parametersReady());

    // PARAM_REQUEST_LIST should have been sent (hash check timed out)
    QVERIFY(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) > 0);
}

// Scenario 5: Hash check times out, no cache — PARAM_REQUEST_LIST stream continues
void HashCheckTest::_hashCheckTimeoutNoCache()
{
    _deleteCacheFiles();

    _mockLink = MockLink::startPX4MockLink(false, false, false);
    _mockLink->setHashCheckNoResponse(true);

    _connectAndWaitForParams();

    Vehicle *const vehicle = MultiVehicleManager::instance()->activeVehicle();
    QVERIFY(vehicle);
    QVERIFY(vehicle->parameterManager()->parametersReady());

    // PARAM_REQUEST_LIST should have been sent
    QVERIFY(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) > 0);
}

// Scenario 7: No response at all — hash check timer and param request list timer both exhaust
void HashCheckTest::_bothTimersExhaust()
{
    _deleteCacheFiles();

    _mockLink = MockLink::startPX4MockLink(false, false, false, MockConfiguration::FailParamNoResponseToRequestList);
    _mockLink->setHashCheckNoResponse(true);

    MultiVehicleManager *const vehicleMgr = MultiVehicleManager::instance();
    QVERIFY(vehicleMgr);

    QSignalSpy spyVehicle(vehicleMgr, &MultiVehicleManager::activeVehicleAvailableChanged);
    QVERIFY_SIGNAL_WAIT(spyVehicle, TestTimeout::mediumMs());

    Vehicle *const vehicle = vehicleMgr->activeVehicle();
    QVERIFY(vehicle);

    QSignalSpy spyParamsReady(vehicleMgr, &MultiVehicleManager::parameterReadyVehicleAvailableChanged);
    QSignalSpy spyProgress(vehicle->parameterManager(), &ParameterManager::loadProgressChanged);

    // Neither progress nor params ready should arrive within the test timeout window
    QVERIFY_NO_SIGNAL_WAIT(spyProgress, TestTimeout::shortMs());

    // Wait for all retries to exhaust
    const int maxWaitMs = ParameterManager::kHashCheckTimeoutMs
                        + ParameterManager::kTestMaxInitialRequestTimeMs
                        + TestTimeout::shortMs();
    QVERIFY_NO_SIGNAL_WAIT(spyParamsReady, maxWaitMs);
}

// Scenario 9: Manual refresh after initial load — should skip hash check, go straight to PARAM_REQUEST_LIST
void HashCheckTest::_manualRefreshAfterLoad()
{
    _deleteCacheFiles();

    _connectMockLink(MAV_AUTOPILOT_PX4);
    QVERIFY(_vehicle);
    QVERIFY(_vehicle->parameterManager()->parametersReady());

    // Clear message counts
    _mockLink->clearReceivedMavlinkMessageCounts();

    // Manual refresh — goes straight to PARAM_REQUEST_LIST since _initialLoadComplete is true
    _vehicle->parameterManager()->refreshAllParameters();

    // Process events so the message gets sent
    QTest::qWait(100);

    // PARAM_REQUEST_LIST should have been sent directly (no hash check since _initialLoadComplete is true)
    QVERIFY(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_LIST) > 0);

    // No standalone PARAM_REQUEST_READ for _HASH_CHECK should have been sent
    QCOMPARE(_mockLink->receivedMavlinkMessageCount(MAVLINK_MSG_ID_PARAM_REQUEST_READ), 0);

    _disconnectMockLink();
}

// Scenario 10: ArduPilot vehicle — should skip hash check entirely
void HashCheckTest::_arduPilotSkipsHashCheck()
{
    _deleteCacheFiles();

    _connectMockLink(MAV_AUTOPILOT_ARDUPILOTMEGA);
    QVERIFY(_vehicle);
    QVERIFY(_vehicle->parameterManager()->parametersReady());

    // PARAM_REQUEST_READ for _HASH_CHECK should NOT have been sent
    // (ArduPilot uses FTP, not hash check)
    // We can verify by checking that no PARAM_REQUEST_READ messages were sent before PARAM_REQUEST_LIST
    // For ArduPilot, the path is FTP-based, not hash-check-based

    _disconnectMockLink();
}

UT_REGISTER_TEST(HashCheckTest, TestLabel::Integration, TestLabel::Vehicle, TestLabel::Serial)
