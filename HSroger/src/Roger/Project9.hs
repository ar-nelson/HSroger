{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project9(State, control, enterParams, initState) where

import           Control.Applicative
import           Control.Arrow        hiding (left, right)
import           Control.Category
import           Control.Monad.Reader
import           Control.Monad.State  hiding (State)
import           Data.Maybe           (fromMaybe)
import           Data.Vec             hiding (get)
import           Prelude              hiding (id, (.))
import           Roger.Math
import           Roger.Project5       (stereoObservation)
import           Roger.Project7       (homePosition)
import           Roger.Robot
import           Roger.TypedLens
import           Roger.Types
import           Roger.Wire

type State = Wire (ReaderT Time IO) Robot Robot

initState ∷ IO State
initState = return pong

newtype BallVelocity = BallVelocity { getBallHistory ∷ [Observation] }

getBallVelocity ∷ BallVelocity → Maybe Vec2D
getBallVelocity (BallVelocity history) = undefined

--------------------------------------------------------------------------------

setup ∷ (Has Robot s, Has BaseSetpoint s, MonadIO m) ⇒ Wire m s s
setup = stateWire (skip (wire (const get) >>> ifWire id)
               >>> action (liftIO (putStrLn "Doing initial setup..."))
               >>> arr lockBaseAndArms
               >>> arr storeStartPosition
               >>> action (put False)
        ) True
      where lockBaseAndArms = mapL $ \r → r { baseSetpoint = basePosition r
                                            , armSetpoint  = armθ r
                                            }
            storeStartPosition s = setL BaseSetpoint (basePosition (getL s)) s

ifTooCloseToCenter ∷ (Has Robot s, MonadIO m) ⇒ Wire m s s
ifTooCloseToCenter = skip $
    arr (getsL (xOf . xyOf . basePosition)) >>> ifWire (>= (-dangerZone))
  where dangerZone = (shoulder armLength + elbow armLength) * 1.1

retreat ∷ (MonadIO m, Has Robot s, Has BaseSetpoint s) ⇒ Wire m s s
retreat = ifWire notConverged
      >>> action (liftIO (putStrLn "Retreating to start position."))
      >>> arr (\s → mapL (\r → r { baseSetpoint = s *. getBaseSetpoint
                                 , armSetpoint  = homePosition
                                 }) s)
  where notConverged l =
          norm (xyR - xyT) > ε_dist || abs (θR - θT) > ε_θ
          where VectorAndAngle { xyOf = xyR, θOf = θR } = baseSetpoint (getL l)
                VectorAndAngle { xyOf = xyT, θOf = θT } = l *. getBaseSetpoint
        ε_dist = 0.1
        ε_θ    = 0.1

takeObservation ∷ (Has Robot s, Has BallVelocity s, Has (Maybe Observation) s
                  , MonadReader Time m, MonadIO m) ⇒ Wire m s s
takeObservation = id --arr foveate
              >>> (arr getL >>> stereoObservation) &&& id
              >>> arr (\(obs, s) → putL (Just obs) s)
              >>> wire popBallHistory
              >>> arr pushBallHistory
  where foveate s         = undefined
        popBallHistory s  = undefined
        pushBallHistory s = (`mapL` s) $ \h →
          BallVelocity (getsL getBallHistory s ++ getBallHistory h)


pong ∷ State
pong = stateWire (loadState >>> (
      setup
  <|> (ifTooCloseToCenter >>> retreat)
  <|> id
  ) >>> storeState) initialState

  where initialState = LensRecord
          `With` BaseSetpoint (VectorAndAngle (Vec2D 0 0) 0)
          `With` (Nothing ∷ Maybe Observation)
          `With` BallVelocity []
        loadState  = wire (\r → liftM (`With` r) get)
        storeState = wire (\(st `With` r) → put st >> return r)

--------------------------------------------------------------------------------

control :: Robot -> State -> Double -> IO (Robot, State)
control roger st t = do (roger', st') ← runReaderT (runWire st roger) (Time t)
                        return (fromMaybe roger roger', st')

enterParams :: State -> IO State
enterParams = return

