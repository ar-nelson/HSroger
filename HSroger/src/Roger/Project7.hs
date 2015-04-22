{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project7( State
                     , control
                     , enterParams
                     , initState
                     , ChaseState(..)
                     , PunchState(..)
                     , homePosition
                     , chase
                     , punch
                     , chasepunch
) where

import           Control.Monad.Except
import           Control.Monad.State  hiding (State)
import           Data.Vec             hiding (get)
import           Roger.Project2       (invArmKinematics)
import           Roger.Project4       (SearchState (..), TrackState (..),
                                       searchtrack, track)
import           Roger.Project5       (stereoObservation)
import           Roger.Robot
import           Roger.Sampling       (PrDist, prRedPrior)
import           Roger.TypedLens
import           Roger.Types

newtype ChaseState = ChaseState { getChaseState ∷ ControlStatus }
newtype PunchState = PunchState { getPunchState ∷ ControlStatus }

instance Show ChaseState where show (ChaseState s) = "ChaseState " ++ show s
instance Show PunchState where show (PunchState s) = "PunchState " ++ show s

type State = LensRecord `With` ChaseState
                        `With` PunchState
                        `With` SearchState
                        `With` TrackState
                        `With` PrDist

initState ∷ IO State
initState = return $ LensRecord `With` ChaseState  Unknown
                                `With` PunchState  Unknown
                                `With` SearchState Unknown
                                `With` TrackState  Unknown
                                `With` prRedPrior

--------------------------------------------------------------------------------

targetDist ∷ Double
targetDist = shoulder armLength + elbow armLength + baseRadius

rε ∷ Double
rε = ballRadius

θε ∷ Double
θε = 0.1

θ'ε ∷ Double
θ'ε = 0.01

homePosition ∷ Pair (ArmPair Double)
homePosition = Pair { left  = ArmPair { shoulder = pi
                                      , elbow    = (-pi) * 0.91
                                      }
                    , right = ArmPair { shoulder = pi
                                      , elbow    = pi * 0.91
                                      }
                    }

--------------------------------------------------------------------------------

chase ∷ (Has Robot s, Has ChaseState s, MonadState s m, MonadIO m) ⇒ m ()
chase = (>>= either (setDebugL ChaseState) (setDebugL ChaseState)) . runExceptT $ do
  st    ← get
  evalStateT track (st `With` TrackState Transient)
  roger    ← getStateL
  maybeObs ← liftIO (stereoObservation roger 0)
  obs      ← maybe (throwError NoReference) (return . obsPos) maybeObs
  let ballOffset            = obs - xyOf (basePosition roger)
      (ballDist, ballAngle) = polar ballOffset
      rError = abs (ballDist - targetDist)
      θError = abs (ballAngle - θOf (basePosition roger))
  mapStateL (\r → r { armSetpoint = homePosition })
  if rError <= rε && θError <= θε
     then return Converged
     else let setPt = VectorAndAngle { xyOf = obs - unpolar targetDist ballAngle
                                     , θOf  = ballAngle
                                     }
          in mapStateL (\r → r { baseSetpoint = setPt })
             -- >> liftIO (putStrLn ("Ball Angle: " ++ show ballAngle))
             >> return Transient

punch ∷ (Has Robot s, Has PunchState s, MonadState s m, MonadIO m) ⇒ m ()
punch = (>>= either giveUp (setDebugL PunchState)) . runExceptT $ do
  roger      ← getStateL
  let Robot{..} = roger
  punchState ← getsStateL getPunchState
  when (punchState == Unknown || punchState == NoReference) $ do
    maybeObs ← liftIO (stereoObservation roger 0)
    obs      ← maybe (throwError NoReference) (return . obsPos) maybeObs
    let (_, ballAngle) = polar (obs - xyOf basePosition)
        target         = obs - unpolar ballRadius ballAngle
    maybe (throwError NoReference)
          (\p → mapStateL $ \r → r { armSetpoint = armSetpoint { right = p } })
          (invArmKinematics roger right target)
  let θError = mapArms (\i → i armθ - i armSetpoint)
  when (right extForce /= Vec2D 0 0)              (throwError Converged)
  when (armMax θError < θε && armMax armθ' < θ'ε) (throwError NoReference)
  return Transient
  where giveUp s = do mapStateL $ \r → r { armSetpoint = homePosition }
                      setDebugL PunchState s
        armMax a = max (shoulder (right a)) (elbow (right a))

chasepunch ∷ ( Has Robot s,      Has SearchState s, Has TrackState s
             , Has ChaseState s, Has PunchState s,  Has PrDist s
             , MonadState s m, MonadIO m
             ) ⇒ m ()
chasepunch = get >>= \st →
  case st *. getTrackState of
    Converged → case st *. getChaseState of
      Unknown   → chase
      Transient → chase
      Converged → case st *. getPunchState of
        Unknown   → punch
        Transient → punch
        _ → do setDebugL SearchState Unknown
               setDebugL TrackState  Unknown
               setDebugL ChaseState  Unknown
               setDebugL PunchState  Unknown
      _ → do setDebugL SearchState Unknown
             setDebugL TrackState  Unknown
             setDebugL ChaseState  Unknown
    _ → searchtrack

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ =
  do st' `With` roger' ← execStateT chasepunch (st `With` roger)
     return (roger', st')

enterParams ∷ State → IO State
enterParams = return

