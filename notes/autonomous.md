ERROR ﻿﻿ 1 ﻿﻿ DifferentialDrive... Output not updated often enough. See https://docs.wpilib.org/motorsafety for more information. ﻿﻿ edu.wpi.first.wpilibj.MotorSafety.check(MotorSafety.java:139) 

Fixed. Needs to have a call to tank drive the entire time during autonomous
