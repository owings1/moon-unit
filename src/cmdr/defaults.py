debug = True

components = {
  'mc': {
    'category': 'motors',
    'classname': 'MotorController',
    'enabled': False,
    'options': {
      'motors': 2,
    },
  },
  'gps': {
    'category': 'geo',
    'classname': 'GPS',
    'enabled': False,
  },
  'imu6': {
    'category': 'inertial',
    'classname': 'IMU6',
    'enabled': False,
    'options': {
      'onboard_i2c': True,
    }
  },
  'imu9': {
    'category': 'inertial',
    'classname': 'IMU9',
    'enabled': False,
  },
}