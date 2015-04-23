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
import           Control.Monad.State  hiding (State)
import           Data.Maybe           (fromMaybe)
import           Data.Vec             hiding (get)
import           Prelude              hiding (id, (.))
import           Roger.Math
import           Roger.Project2       (invArmKinematics)
import           Roger.Project4       (searchtrack, track)
import           Roger.Project5       (stereoObservation)
import           Roger.Robot
import           Roger.TypedLens
import           Roger.Types
import           Roger.Wire

type State = Wire (ReaderT Time IO) Robot Robot

data CPState = SearchTrack | Chase | Punch deriving Eq

initState ∷ IO State
initState = return (chasepunch >>^ fst)

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

chase = (track                    >>^ setL ArmSetpoint homePosition . fst)
    >>> (id &&& stereoObservation >>^ updateSetpoint)

  where updateSetpoint (roger@Robot{..}, Observation{ obsPos = obs })
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
      ⇒ Wire (StateT ControlStatus m) Robot Robot
punch = lockBasePosition
    >>> (ifAlreadySetTarget >>> checkIfDone)
    <|> (id &&& stereoObservation >>> setPunchTarget >>> checkIfDone)
    <|> giveUp

  where ifAlreadySetTarget =
          skip (wire (const get) >>> ifWire (`elem` [Transient, Converged]))

        setPunchTarget = maybeWire $ arr $
          \(roger@Robot{..}, Observation{ obsPos = obs }) →
            let (_, ballAngle) = polar (obs - xyOf basePosition)
                target         = obs - unpolar (ballRadius / 2) ballAngle
            in liftM (\setp → roger {armSetpoint = armSetpoint {right = setp}})
                     (invArmKinematics roger right target)

        checkIfDone = checkForCollision
                  <|> (ifWire armStillMoving >>> action (put Transient))

        checkForCollision = ifWire (\r → right (extForce r) /= Vec2D 0 0)
                        >>> arr moveArmsToHomePosition
                        >>> action (put Converged)

        armStillMoving Robot{..} = armMax θError > θε || armMax armθ' > θ'ε
          where θError   = mapArms (\i → i armθ - i armSetpoint)
                armMax a = max (shoulder (right a)) (elbow (right a))

        lockBasePosition = arr (\r@Robot{..} → r {baseSetpoint = basePosition})

        giveUp = moveArmsToHomePosition ^>> action (put NoReference)

chasepunch ∷ (MonadReader Time m, MonadIO m)
           ⇒ Wire m Robot (Robot, ControlStatus)
chasepunch = stateWire ((  (isState Punch       >>> doPunch)
                       <|> (isState Chase       >>> doChase)
                       <|> (isState SearchTrack >>> doSearchTrack)
                        ) >>> second (second (wire put)) >>^ second fst
                       ) SearchTrack

  where doSearchTrack = searchtrack <|> (id &&& pure NoReference)
                    >>^ second (id &&& transition)
          where transition Converged = Chase
                transition _         = SearchTrack

        doChase = chase <|> (id &&& pure NoReference)
              >>^ second (id &&& transition)
          where transition Converged   = Punch
                transition NoReference = SearchTrack
                transition _           = Chase

        doPunch = stateWire ( punch <|> id
                          >>> id &&& wire (const get)
                          >>> second (id &&& wire transition)) Unknown
          where transition Converged   = put Unknown >> return Chase
                transition NoReference = return SearchTrack
                transition _           = return Punch

        isState s = skip (wire (const get) >>> ifWire (== s))

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st t = do (roger', st') ← runReaderT (runWire st roger) (Time t)
                        return (fromMaybe roger roger', st')

enterParams ∷ State → IO State
enterParams = return

