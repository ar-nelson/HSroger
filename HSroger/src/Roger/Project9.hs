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
import qualified Data.Vec             as V
import           Prelude              hiding (id, (.))
import           Roger.Math
import           Roger.Project2       (invArmKinematics)
import           Roger.Project3       (avgRedWire, imageCoordToAngle)
import           Roger.Project5       (stereoObservation)
import           Roger.Project6       (blankEstimate, kalmanFilter)
import           Roger.Project7       (homePosition)
import           Roger.Robot
import           Roger.Types
import           Roger.Wire
import           System.IO
import           System.Random

type State = (Wire (ReaderT Time IO) Robot (Robot, Estimate), Estimate)

{-
data Sample = Sample { smpObs  ∷ Observation
                     , smpDist ∷ Double
                     , smpVel  ∷ V.Vec2D
                     }
-}

defObs ∷ Observation
defObs = Observation { obsPos = V.Vec2D 0 0
                     , obsCov = mat22 (0, 0, 0, 0)
                     , obsTime = 0
                     }

getEstimate ∷ State → Estimate
getEstimate = snd

{-
getVelocity ∷ State → V.Vec2D
getVelocity st = maybe (V.Vec2D 0 0) smpVel (snd st)
-}

initState ∷ IO State
initState = return (pong, blankEstimate)

--------------------------------------------------------------------------------

bufferDistance ∷ Double
bufferDistance = shoulder armLength + elbow armLength
{-
sampleLifetime ∷ Double
sampleLifetime = 0.01 -- seconds
-}
baseHome ∷ VectorAndAngle
baseHome = VectorAndAngle { xyOf = V.Vec2D (-4.0) 0.0
                          , θOf  = 0.0
                          }

--------------------------------------------------------------------------------

eyeController ∷ (MonadReader Time m, MonadIO m)
              ⇒ Wire m Robot (Robot, Maybe Observation)
eyeController = ballSeenPath <|> ballUnseenPath

  where ballSeenPath = proc roger →
          do roger'   ← foveate           -< roger
             obs      ← stereoObservation -< roger'
             {-samples' ← cleanupSamples    -< samples
             let tempSample = Sample { smpObs  = obs
                                     , smpDist = dist roger obs
                                     , smpVel  = V.Vec2D 0 0
                                     }
                 sample = tempSample { smpVel = vel }
                 vel    = guessBallVelocity (tempSample:samples')
             returnA -< ((roger', Just sample), sample:samples')-}
             returnA -< (roger', Just obs)

        ballUnseenPath = proc roger →
          do roger'   ← lookForBall <|> id -< roger
             --samples' ← cleanupSamples     -< samples
             returnA -< (roger', Nothing)

        --dist Robot{..} Observation{..} = V.norm (xyOf basePosition - obsPos)

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
{-
cleanupSamples ∷ (MonadReader Time m) ⇒ Wire m [Sample] [Sample]
cleanupSamples = wire $ \samples →
  do now ← asks timeSeconds
     return $ take 32 (filter (notOld now) samples)
  where notOld now Sample { smpObs = Observation { obsTime = t } } =
          now < t + sampleLifetime

guessBallVelocity ∷ [Sample] → V.Vec2D
guessBallVelocity []   = V.Vec2D 0 0
guessBallVelocity [_]  = V.Vec2D 0 0
guessBallVelocity smps = mean (zipWith timeDiff smps (tail smps))
  where timeDiff a b = m // s where m = obsPos  (smpObs a) - obsPos  (smpObs b)
                                    s = obsTime (smpObs a) - obsTime (smpObs b)
        mean xs = let (t, n) = foldl' (\(b,c) a → (a+b,c+1)) (V.Vec2D 0 0, 0) xs
                  in t // n
        vector // scalar = V.map (/ scalar) vector
-}
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
retreat = ifWire notConverged >>> proc roger →
  do debugStr -< "Retreating to start position."
     returnA  -< roger { baseSetpoint = baseHome
                       , armSetpoint  = homePosition
                       }
  where notConverged roger =
          V.norm (xyR - xyT) > ε_dist || abs (θR - θT) > ε_θ
          where VectorAndAngle { xyOf = xyR, θOf = θR } = baseSetpoint roger
                VectorAndAngle { xyOf = xyT, θOf = θT } = baseHome
                ε_dist = 0.1
                ε_θ    = 0.1

intercept ∷ (MonadIO m) ⇒ Wire m (Robot, (Maybe Observation, Estimate)) Robot
intercept = proc (roger, (maybeObs, Estimate{..})) →
  do Observation{..} ← maybeWire id -< maybeObs
     let vel = V.drop V.n2 estState
         pt  = obsPos + (V.map (* 0.1) vel) + V.Vec2D (ballRadius*2) 0 -- (V.map (* (ballRadius * 2)) (V.normalize vel))
     ifWire (< 0) -< xOf obsPos
     ifWire (< 0) -< xOf vel
     returnA -< safeSetBaseTarget roger pt

swingArm ∷ (MonadIO m) ⇒ Wire m (Robot, (Maybe Observation, Estimate)) Robot
swingArm = proc (roger@Robot{..}, (maybeObs, Estimate{..})) →
  let ballPos     = maybe (V.take V.n2 estState) obsPos maybeObs
      ballVel     = V.drop V.n2 estState
      rogerPos    = xyOf basePosition
      shoulderPos = mapPair $ \i →
        rogerPos + unpolar (i armOffset) (θOf basePosition + pi/4.0)
      relShoulderPos = mapPair (\i → i shoulderPos - ballPos)
      shoulderIntersect = mapPair (\i → vectorProj (i relShoulderPos) ballVel)
  in do ifWire (< shoulder armLength + elbow armLength + ballRadius * 2) -< V.norm (rogerPos - ballPos)
        if V.norm (left shoulderIntersect) < V.norm (right shoulderIntersect)
           then safeSetArmTarget left -<
                   (roger, (rogerPos + left shoulderIntersect) - V.Vec2D (ballRadius*2) 0)
           else safeSetArmTarget right -<
                   (roger, (rogerPos + right shoulderIntersect) - V.Vec2D (ballRadius*2) 0)
  where vectorProj x0 v = V.map (* (V.dot x0 v / sq (V.norm v))) v

clampSafeVec ∷ Double → V.Vec2D → V.Vec2D
clampSafeVec space v = V.Vec2D (max (minX+space) (min (-space) (xOf v)))
                               (max (minY+space) (min (maxY-space) (yOf v)))

safeSetBaseTarget ∷ Robot → V.Vec2D → Robot
safeSetBaseTarget r p = r { baseSetpoint = VectorAndAngle
                              { xyOf = p'
                              , θOf  = snd (polar (p' - xyOf (basePosition r)))
                              }
                          }
  where p' = clampSafeVec (shoulder armLength) p

safeSetArmTarget ∷ Monad m ⇒ (∀ a. Pair a → a) → Wire m (Robot, V.Vec2D) Robot
safeSetArmTarget a = maybeWire . arr $ \(r, p) →
  a (assignFn r) `fmap` invArmKinematics r a (clampSafeVec ballRadius p)
    where assignFn r = Pair
            { left  = \p → r { armSetpoint = (armSetpoint r) { left  = p } }
            , right = \p → r { armSetpoint = (armSetpoint r) { right = p } }
            }

--------------------------------------------------------------------------------

pong ∷ Wire (ReaderT Time IO) Robot (Robot, Estimate)
pong = proc roger0 →
  do (roger1, obs) ← eyeController -< roger0
     est           ← kalmanFilter  -< roger1
     roger2 ← setup . arr fst
          -- <|> swingArm
          <|> intercept
          <|> retreat . arr fst
          <|> arr fst -< (roger1, (obs, est))
     action (liftIO (hFlush stdout)) -< ()
     returnA -< (roger2, est)

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger (w, _) t =
  do (tuple, w') ← runReaderT (runWire w roger) (Time t)
     case tuple of
       Just (roger', est) → return (roger', (w', est))
       Nothing            → return (roger,  (w', blankEstimate))

enterParams ∷ State → IO State
enterParams = return

