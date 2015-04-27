{-# LANGUAGE Arrows                #-}
{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RankNTypes            #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project9( State
                     , control
                     , enterParams
                     , initState
                     , getEstimate
) where

import           Control.Applicative
import           Control.Arrow        hiding (left, right)
import           Control.Category
import           Control.Monad.Reader
import           Data.Foldable
import           Data.Maybe           (catMaybes)
import qualified Data.Vec             as V
import           Prelude              hiding (id, (.))
import           Roger.Math
import           Roger.Project2       (invArmKinematics)
import           Roger.Project3       (avgRedWire, imageCoordToAngle)
import           Roger.Project5       (stereoObservation)
import           Roger.Project6       (blankEstimate)
import           Roger.Project7       (homePosition)
import           Roger.Robot
import           Roger.Types
import           Roger.Wire
import           System.IO
import           System.Random

type State = (Wire (ReaderT Time IO) Robot (Robot, Estimate), Estimate)

getEstimate ∷ State → Estimate
getEstimate = snd


initState ∷ IO State
initState = return (pong, blankEstimate)

baseHome ∷ VectorAndAngle
baseHome = VectorAndAngle { xyOf = V.Vec2D (-4.0) 0.0
                          , θOf  = 0.0
                          }

--------------------------------------------------------------------------------

eyeController ∷ (MonadReader Time m, MonadIO m)
              ⇒ Wire m Robot (Robot, (Maybe Observation, V.Vec2D))
eyeController = stateWire (Nothing, replicate 5 Nothing)
                          (ballSeenPath <|> ballUnseenPath)
                          >>^ second (second avgVels)

  where ballSeenPath = proc (roger, (lastObs, vels)) →
          do roger'   ← foveate           -< roger
             obs      ← stereoObservation -< roger'
             let mObs = lastObs >>= \o → if V.norm (obsPos obs - obsPos o) > ballRadius
                                            then Nothing else Just obs
                 vel = liftM2 obsDiff mObs lastObs
                 adjVel = liftM (V.map (/ max 1 (V.norm (xyOf (basePosition roger) - obsPos obs)))) vel
                 obsDiff a b = V.map (/ 0.001) (obsPos a - obsPos b)
             returnA -< (roger', (Just obs, tail vels ++ [adjVel]))

        ballUnseenPath = proc (roger, (_, vels)) →
          do roger'   ← lookForBall <|> id -< roger
             returnA -< (roger', (Nothing, tail vels ++ [Nothing]))

        avgVels vs = let (t, n) = foldl' (\(b,c) a → (a+b,c+1)) (V.Vec2D 0 0, 0)
                                                                (catMaybes vs)
                     in V.map (/ n) t

foveate ∷ (MonadIO m) ⇒ Wire m Robot Robot
foveate = proc roger@Robot{..} →
  do avgRed ← avgRedWire -< roger
     let eyes = mapPair (\i → i eyeθ + imageCoordToAngle (i avgRed))
     returnA -< roger { eyeSetpoint = eyes }

lookForBall ∷ (MonadReader Time m, MonadIO m) ⇒ Wire m Robot Robot
lookForBall = proc roger →
  do rnd ← randomAngle . interval 0.01 -< ()
     returnA -< roger { eyeSetpoint = mapPair (const rnd) }
  where randomAngle = wire . const . liftIO $ randomRIO (-(pi/2.0), pi/2.0)

--------------------------------------------------------------------------------

setup ∷ (MonadIO m) ⇒ Wire m Robot Robot
setup = localStateWire True $ proc (roger, notYetRun) →
  do ifWire id -< notYetRun
     debugMsg "Start position: " -< basePosition roger
     let roger' = roger { baseSetpoint = basePosition roger
                        , armSetpoint  = armθ roger
                        }
     returnA -< (roger', False)

retreat ∷ (MonadIO m) ⇒ Wire m Robot Robot
retreat = ifWire notConverged >>> arr (\r → r { baseSetpoint = baseHome
                                              , armSetpoint  = homePosition
                                              })
  where notConverged roger =
          V.norm (xyR - xyT) > ε_dist || abs (θR - θT) > ε_θ
          where VectorAndAngle { xyOf = xyR, θOf = θR } = baseSetpoint roger
                VectorAndAngle { xyOf = xyT, θOf = θT } = baseHome
                ε_dist = 0.1
                ε_θ    = 0.1

intercept ∷ (MonadIO m) ⇒ Wire m (Robot, (Maybe Observation, V.Vec2D)) Robot
intercept = proc (roger, (maybeObs, vel)) →
  do Observation{..} ← maybeWire id -< maybeObs
     let pt  = obsPos + V.map (*0.1) vel
     ifWire (< 0) -< xOf obsPos
     ifWire (< 0) -< xOf vel
     returnA -< safeSetBaseTarget roger pt

swingArm ∷ (MonadIO m) ⇒ Wire m (Robot, (Maybe Observation, V.Vec2D)) Robot
swingArm = proc (roger@Robot{..}, (maybeObs, vel)) →
  do obs ← maybeWire id -< maybeObs
     let ballPos     = obsPos obs
         rogerPos    = xyOf basePosition
         shoulderPos = mapPair $ \i →
           rogerPos + unpolar (i armOffset) (θOf basePosition + pi/4.0)
         relShoulderPos = mapPair (\i → i shoulderPos - ballPos)
         shoulderIntersect = mapPair (\i → vectorProj (i relShoulderPos) vel)
     ifWire (< shoulder armLength + elbow armLength + ballRadius * 2) -< V.norm (rogerPos - ballPos)
     if V.norm (left shoulderIntersect) < V.norm (right shoulderIntersect)
        then safeSetArmTarget left -<
               (roger, (ballPos + left shoulderIntersect) - V.Vec2D ballRadius 0)
        else safeSetArmTarget right -<
               (roger, (ballPos + right shoulderIntersect) - V.Vec2D ballRadius 0)
  where vectorProj x0 v = V.map (* (V.dot x0 v / sq (V.norm v))) v

clampSafeVec ∷ Double → V.Vec2D → V.Vec2D
clampSafeVec space v = V.Vec2D (max (minX+space) (min (-space) (xOf v)))
                               (max (minY+space) (min (maxY-space) (yOf v)))

safeSetBaseTarget ∷ Robot → V.Vec2D → Robot
safeSetBaseTarget r p = r { baseSetpoint = VectorAndAngle
                              { xyOf = p'
                              , θOf  = clampedθ
                              }
                          }
  where p' = clampSafeVec (shoulder armLength + elbow armLength + baseRadius) p
        θ  = snd (polar (p' - xyOf (basePosition r)))
        clampedθ = if θ < (-(pi/2.0)) || θ > pi/2.0
                       then θ + pi
                       else θ

safeSetArmTarget ∷ Monad m ⇒ (∀ a. Pair a → a) → Wire m (Robot, V.Vec2D) Robot
safeSetArmTarget a = maybeWire . arr $ \(r, p) →
  a (assignFn r) `fmap` invArmKinematics r a (clampSafeVec (elbow armLength) p)
    where assignFn r = Pair
            { left  = \p → r { armSetpoint = (armSetpoint r) { left  = p } }
            , right = \p → r { armSetpoint = (armSetpoint r) { right = p } }
            }

--------------------------------------------------------------------------------

pong ∷ Wire (ReaderT Time IO) Robot (Robot, Estimate)
pong = proc roger0 →
  do (roger1, (obs, vel)) ← eyeController -< roger0
     roger2 ← setup . arr fst
          <|> (proc (r, ov) → do r' ← swingArm <|> arr fst -< (r, ov)
                                 intercept -< (r', ov))
          <|> intercept
          <|> (interval 0.1 >>> retreat) . arr fst
          <|> arr fst -< (roger1, (obs, vel))
     action (liftIO (hFlush stdout)) -< ()
     returnA -< (roger2, makeEstimate obs vel)
  where makeEstimate Nothing _ = blankEstimate
        makeEstimate (Just Observation {..}) vel = blankEstimate
          { estState = V.Vec4D (xOf obsPos) (yOf obsPos) (xOf vel) (yOf vel) }

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger (w, _) t =
  do (tuple, w') ← runReaderT (runWire w roger) (Time t)
     case tuple of
       Just (roger', est) → return (roger', (w', est))
       Nothing            → return (roger,  (w', blankEstimate))

enterParams ∷ State → IO State
enterParams = return

