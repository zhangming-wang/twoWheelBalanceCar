#pragma once

enum ServiceType {
    Idel = 0,
    Restart,
    Brake,
    StopMove,
    MoveFront,
    MoveBack,
    MoveLeft,
    MoveRight,

    SetNoBalanceMode,
    SetStaticBalanceMode,
    SetDynamicBalanceMode,

    CalibrateMPU,
    SetOffsetMPU,

    SetSpeedPercent,
    SetSpeedPlanState,

    ReadParams,
    WriteParams,
    SaveParams,

    ReadSettings,
    WriteSettings,
    SaveSettings,
};

enum MotionMode {
    NoBalanceMode = 0,
    StaticBalanceMode = 1,
    DynamicBalanceMode = 2
};
