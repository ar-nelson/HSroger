{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project7( State
                     , control
                     , enterParams
                     , reset
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
import           Roger.Project5       (Observation (..), stereoObservation)
import           Roger.Robot
import           Roger.Sampling       (PrDist, prRedPrior)
import           Roger.TypedLens
import           Roger.Types

newtype ChaseState = ChaseState { getChaseState ∷ ControlStatus }
newtype PunchState = PunchState { getPunchState ∷ ControlStatus }

type State = LensRecord `With` ChaseState
                        `With` PunchState
                        `With` SearchState
                        `With` TrackState
                        `With` PrDist

initState ∷ IO State
initState = return $ LensRecord `With` ChaseState  UNKNOWN
                                `With` PunchState  UNKNOWN
                                `With` SearchState UNKNOWN
                                `With` TrackState  UNKNOWN
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

chase ∷ (Lens Robot s, Lens ChaseState s, MonadState s m, MonadIO m) ⇒ m ()
chase = (>>= either (ChaseState *=) (ChaseState *=)) . runExceptT $ do
  st    ← get
  evalStateT track (st `With` TrackState UNKNOWN)
  roger    ← lgetSt
  maybeObs ← liftIO (stereoObservation roger)
  obs      ← maybe (throwError NO_REFERENCE) (return . obsPos) maybeObs
  let ballOffset            = obs - xyOf (basePosition roger)
      (ballDist, ballAngle) = polar ballOffset
      rError = abs (ballDist - targetDist)
      θError = abs (ballAngle - θOf (basePosition roger))
  if rError <= rε && θError <= θε
     then lputSt roger { armSetpoint = homePosition }
          >> return CONVERGED
     else lputSt roger { armSetpoint  = homePosition
                       , baseSetpoint = VectorAndAngle
                           { xyOf = obs - unpolar targetDist ballAngle
                           , θOf  = ballAngle
                           }
                       }
          >> liftIO (putStrLn ("Ball Angle: " ++ show ballAngle))
          >> return TRANSIENT

punch ∷ (Lens Robot s, Lens PunchState s, MonadState s m, MonadIO m) ⇒ m ()
punch = (>>= either giveUp (PunchState *=)) . runExceptT $ do
  roger      ← lgetSt
  punchState ← lgetsSt getPunchState
  when (punchState == UNKNOWN || punchState == NO_REFERENCE) $ do
    maybeObs ← liftIO (stereoObservation roger)
    obs      ← maybe (throwError NO_REFERENCE) (return . obsPos) maybeObs
    let (_, ballAngle) = polar (obs - xyOf (basePosition roger))
        target         = obs - unpolar ballRadius ballAngle
    maybe (throwError NO_REFERENCE)
          (\p → lputSt roger {armSetpoint = (armSetpoint roger){right = p}})
          (invArmKinematics roger right target)
  let θError = mapArms (\i → i (armθ roger) - i (armSetpoint roger))
  when (right (extForce roger) /= Vec2D 0 0) $
    throwError CONVERGED
  when (armMax θError < θε && armMax (armθ' roger) < θ'ε) $
    throwError NO_REFERENCE
  return TRANSIENT
  where giveUp s = do roger ← lgetSt
                      lputSt roger { armSetpoint = homePosition }
                      PunchState *= s
        armMax a = max (shoulder (right a)) (elbow (right a))

chasepunch ∷ ( Lens Robot s,      Lens SearchState s, Lens TrackState s
             , Lens ChaseState s, Lens PunchState s,  Lens PrDist s
             , MonadState s m, MonadIO m
             ) ⇒ m ()
chasepunch = get >>= \st →
  case st *. getTrackState of
    CONVERGED → case st *. getChaseState of
      UNKNOWN   → chase
      TRANSIENT → chase
      CONVERGED → case st *. getPunchState of
        UNKNOWN   → punch
        TRANSIENT → punch
        _ → do SearchState *= UNKNOWN
               TrackState  *= UNKNOWN
               ChaseState  *= UNKNOWN
               PunchState  *= UNKNOWN
      _ → do SearchState *= UNKNOWN
             TrackState  *= UNKNOWN
             ChaseState  *= UNKNOWN
    _ → searchtrack

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ =
  do st' `With` roger' ← execStateT chasepunch (st `With` roger)
     return (roger', st')

enterParams ∷ State → IO State
enterParams = return

reset ∷ Robot → State → IO (Robot, State)
reset r _ = initState >>= \s → return (r, s)

