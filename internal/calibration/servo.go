package calibration

// ServoReading is one raw sample from a servo.
type ServoReading struct {
	PositionRaw uint16
	CurrentRaw  uint16
	OK          bool
}

// ServoClient is the calibration package's view of a servo, satisfied by the
// servobus adapter (internal/servobus) in production and by fakes in tests.
type ServoClient interface {
	SetMode(id uint32, mode uint32) error
	SetTorqueLimit(id uint32, raw uint16) error
	SetOvercurrentLimit(id uint32, raw uint16) error
	SetAngleLimits(id uint32, min, max uint16) error
	SetGoalSpeed(id uint32, raw uint16) error
	EnableTorque(id uint32) error
	DisableTorque(id uint32) error
	SetGoalPosition(id uint32, raw uint16) error
	Read(id uint32) (ServoReading, error)
}
