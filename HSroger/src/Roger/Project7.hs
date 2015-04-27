{-# LANGUAGE Arrows                #-}
{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project7( State
                     , control
                     , enterParams
                     , initState
                     , homePosition
                     , chase
                     , punch
                     , chasepunch
) where

import           Control.Applicative
import           Control.Arrow        hiding (left, right)
import           Control.Category
import           Control.Monad.Reader
import           Data.Maybe           (fromMaybe)
import           Data.Vec             hiding (get)
import           Prelude              hiding (id, (.))
import           Roger.Math
import           Roger.Project2       (invArmKinematics)
import           Roger.Project4       (searchtrack, track)
import           Roger.Project5       (stereoObservation)
import           Roger.Robot
import           Roger.Types
import           Roger.Wire

type State = Wire (ReaderT Time IO) Robot Robot

data CPState = SearchTrack | Chase | Punch deriving Eq

initState ∷ IO State
initState = return (setup <|> (chasepunch >>^ fst))

--------------------------------------------------------------------------------

punchDist ∷ Double
punchDist = shoulder armLength + elbow armLength

setptDist ∷ Double
setptDist = ballRadius

baseθε ∷ Double
baseθε = 0.4

θ'ε ∷ Double
θ'ε = 0.01

θε ∷ Double
θε = 0.1

homePosition ∷ Pair (ArmPair Double)
homePosition = Pair { left  = ArmPair { shoulder = pi
                                      , elbow    = (-pi) * 0.91
                                      }
                    , right = ArmPair { shoulder = pi
                                      , elbow    = pi * 0.91
                                      }
                    }

moveArmsToHomePosition ∷ Robot → Robot
moveArmsToHomePosition roger = roger { armSetpoint = homePosition }

--------------------------------------------------------------------------------

chase ∷ (MonadReader Time m, MonadIO m) ⇒ Wire m Robot (Robot, ControlStatus)

chase = proc roger → do (roger', _) ← track -< roger
                        obs ← stereoObservation -< roger'
                        returnA -< updateSetpoint roger' obs

  where updateSetpoint roger@Robot{..} Observation{ obsPos = obs }
          | ballDist <= punchDist &&
            ballDist >= setptDist &&
            θError   <= baseθε     = (roger, Converged)
          | otherwise              = (roger { baseSetpoint = sp }, Transient)

          where (ballDist, ballAngle) = polar (obs - xyOf basePosition)
                θError = abs (ballAngle - θOf basePosition)
                sp = VectorAndAngle { xyOf = obs - unpolar setptDist ballAngle
                                    , θOf  = ballAngle
                                    }

punch ∷ (MonadReader Time m, MonadIO m)
      ⇒ Wire m (Robot, ControlStatus) (Robot, ControlStatus)
punch = (<|> giveUp) $ proc (origRoger, st) →
  do let roger = origRoger { baseSetpoint = basePosition origRoger }
     if st == Transient || st == Converged
        then checkIfDone -< roger
        else do obs    ← stereoObservation -< roger
                roger' ← setPunchTarget    -< (roger, obs)
                checkIfDone -< roger'

  where setPunchTarget = maybeWire $ arr $
          \(roger@Robot{..}, Observation{ obsPos = obs }) →
            let (_, ballAngle) = polar (obs - xyOf basePosition)
                target         = obs - unpolar (ballRadius / 2) ballAngle
            in liftM (\setp → roger {armSetpoint = armSetpoint {right = setp}})
                     (invArmKinematics roger right target)

        checkIfDone = checkForCollision <|> proc roger →
          if armStillMoving roger
             then returnA -< (roger, Transient)
             else empty -< ()

        checkForCollision = proc roger →
          if right (extForce roger) /= Vec2D 0 0
             then returnA -< (moveArmsToHomePosition roger, Converged)
             else empty -< ()

        armStillMoving Robot{..} = armMax θError > θε || armMax armθ' > θ'ε
          where θError   = mapArms (\i → i armθ - i armSetpoint)
                armMax a = max (shoulder (right a)) (elbow (right a))

        giveUp = proc (roger, _) →
          returnA -< (moveArmsToHomePosition roger, NoReference)

chasepunch ∷ (MonadReader Time m, MonadIO m)
           ⇒ Wire m Robot (Robot, ControlStatus)
chasepunch = localStateWire SearchTrack $ proc (roger, st) → case st of
  Punch       → doPunch       -< roger
  Chase       → doChase       -< roger
  SearchTrack → doSearchTrack -< roger

  where doPunch = stateWire Unknown (punch <|> (fst ^>> noRef)) >>^ transition
          where transition (r, Converged)   = ((r, Unknown), Chase)
                transition (r, NoReference) = ((r, NoReference), SearchTrack)
                transition t                = (t, Punch)

        doChase = proc roger →
          do (roger', st) ← chase <|> noRef -< roger
             returnA -< ((roger', st), transition st)
          where transition Converged   = Punch
                transition NoReference = SearchTrack
                transition _           = Chase

        doSearchTrack = proc roger →
          do (roger', st) ← searchtrack <|> noRef -< roger
             returnA -< ((roger', st), transition st)
          where transition Converged = Chase
                transition _         = SearchTrack

        noRef = id &&& pure NoReference

setup ∷ (MonadIO m) ⇒ Wire m Robot Robot
setup = localStateWire True $ proc (roger, notYetRun) →
  do ifWire id -< notYetRun
     debugStr  -< "Doing initial setup..."
     let roger' = roger { baseSetpoint = basePosition roger
                        , armSetpoint  = armθ roger
                        }
     returnA   -< (roger', False)

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st t = do (roger', st') ← runReaderT (runWire st roger) (Time t)
                        return (fromMaybe roger roger', st')

enterParams ∷ State → IO State
enterParams = return

