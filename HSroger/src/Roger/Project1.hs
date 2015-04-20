{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE UnicodeSyntax   #-}

module Roger.Project1(control) where

import           Data.Vec
import           Roger.Robot
import           Roger.Types

data Gains = Gains { _Kp ∷ Double, _Kd ∷ Double }

eyeGains ∷ Gains
eyeGains = Gains { _Kp = 5.0, _Kd = 0.04 }

armGains ∷ Gains
armGains = Gains { _Kp = 180.0, _Kd = 6.0 }

baseTransGains ∷ Gains
baseTransGains = Gains { _Kp = 726.0, _Kd =  66.0 }

baseRotGains ∷ Gains
baseRotGains = Gains { _Kp = 726.0, _Kd = 40.55645 }

baseJT ∷ Pair (Double, Double)
baseJT = Pair { left  = (1.0 / 2.0, -1.0 / (2.0 * axleRadius))
              , right = (1.0 / 2.0,  1.0 / (2.0 * axleRadius))
              }

--------------------------------------------------------------------------------

anglePDController ∷ Gains → Double → Double → Double → Double
anglePDController gains setθ θ θ' =
  (_Kp gains * θerror) + (_Kd gains * θ'error)
  where θerror  = clampAngle (setθ - θ)
        θ'error = -θ'

transPDController ∷ Gains → Vec2D → VectorAndAngle → Vec2D → Double
transPDController gains setPoint pos vel =
  _Kp gains * transError - _Kd gains * transVel
  where baseError  = setPoint - xyOf pos
        transError = project baseError (θOf pos)
        transVel   = project vel (θOf pos)
        project (Vec2D x y) θ = x * cos θ + y * sin θ

control ∷ Robot → Double → IO Robot
control Robot{..} _ = return Robot{..}
  where eyeTorque = mapPair $ \i -> anglePD eyeGains eyeSetpoint eyeθ eyeθ' i
        armTorque = mapArms $ \i -> anglePD armGains armSetpoint armθ armθ' i
        wheelTorque = mapPair $ \i → fst (i baseJT) * fx + snd (i baseJT) * mz
        fx = transPDController baseTransGains (xyOf baseSetpoint) basePosition (xyOf baseVelocity)
        mz = anglePD baseRotGains baseSetpoint basePosition baseVelocity θOf
        anglePD gains setp cur cur' fn =
          anglePDController gains (fn setp) (fn cur) (fn cur')

