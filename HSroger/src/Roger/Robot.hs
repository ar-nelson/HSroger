{-# LANGUAGE UnicodeSyntax #-}

module Roger.Robot( axleRadius
                  , armLength
                  , armOffset
                  , focalLength
                  , Robot(..)
) where

import           Control.Applicative
import           Data.Vec
import           Foreign.Storable
import           Roger.Types

axleRadius ∷ Double
axleRadius = 0.20

armLength ∷ ArmPair Double
armLength  = ArmPair { shoulder = 0.5, elbow = 0.5 }

armOffset ∷ Pair Double
armOffset  = Pair { left = -0.18, right = 0.18 }

focalLength :: Double
focalLength = 64 -- pixels

--------------------------------------------------------------------------------

data Robot = Robot { eyeθ         ∷ Pair Double
                   , eyeθ'        ∷ Pair Double
                   , image        ∷ Pair Image
                   , armθ         ∷ Pair (ArmPair Double)
                   , armθ'        ∷ Pair (ArmPair Double)
                   , extForce     ∷ Pair Vec2D
                   , basePosition ∷ VectorAndAngle
                   , baseVelocity ∷ VectorAndAngle
                   , wheelθ'      ∷ Pair Double
                   , eyeTorque    ∷ Pair Double
                   , armTorque    ∷ Pair (ArmPair Double)
                   , wheelTorque  ∷ Pair Double
                   , baseSetpoint ∷ VectorAndAngle
                   , armSetpoint  ∷ Pair (ArmPair Double)
                   , eyeSetpoint  ∷ Pair Double
                   }

instance Storable Robot where
  sizeOf _ = 494960 -- Obtained via C2HS.
  alignment _ = 4
  peek p = Robot <$> peekByteOff p offset'eye_theta
                 <*> peekByteOff p offset'eye_theta_dot
                 <*> peekByteOff p offset'image
                 <*> peekByteOff p offset'arm_theta
                 <*> peekByteOff p offset'arm_theta_dot
                 <*> peekByteOff p offset'ext_force
                 <*> peekByteOff p offset'base_position
                 <*> peekByteOff p offset'base_velocity
                 <*> peekByteOff p offset'wheel_theta_dot
                 <*> peekByteOff p offset'eye_torque
                 <*> peekByteOff p offset'arm_torque
                 <*> peekByteOff p offset'wheel_torque
                 <*> peekByteOff p offset'base_setpoint
                 <*> peekByteOff p offset'arm_setpoint
                 <*> peekByteOff p offset'eyes_setpoint
  poke p robot = do pokeByteOff p offset'eye_torque    (eyeTorque robot)
                    pokeByteOff p offset'arm_torque    (armTorque robot)
                    pokeByteOff p offset'wheel_torque  (wheelTorque robot)
                    pokeByteOff p offset'base_setpoint (baseSetpoint robot)
                    pokeByteOff p offset'arm_setpoint  (armSetpoint robot)
                    pokeByteOff p offset'eyes_setpoint (eyeSetpoint robot)

-- Offsets obtained via C2HS. DO NOT EDIT.
offset'eye_theta       = 0
offset'eye_theta_dot   = 16
offset'image           = 32
offset'arm_theta       = 3104
offset'arm_theta_dot   = 3136
offset'ext_force       = 3168
offset'base_position   = 3200
offset'base_velocity   = 3224
offset'wheel_theta_dot = 3248
offset'eye_torque      = 3264
offset'arm_torque      = 3280
offset'wheel_torque    = 3312
offset'base_setpoint   = 494888
offset'arm_setpoint    = 494912
offset'eyes_setpoint   = 494944

