{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE TypeOperators   #-}
{-# LANGUAGE UnicodeSyntax   #-}

module Roger.Project5( State
                     , control
                     , enterParams
                     , initState
                     , stereoObservation
                     , getObservation
) where

import           Control.Monad
import           Control.Monad.State hiding (State)
import           Data.Maybe          (fromMaybe)
import           Data.Vec
import           Prelude             hiding (map, take)
import           Roger.Project3      (computeAverageRedPixel, imageCoordToAngle)
import           Roger.Project4      (SearchState (..), TrackState (..),
                                      searchtrack)
import           Roger.Robot
import           Roger.Sampling      (PrDist, prRedPrior)
import           Roger.TypedLens
import           Roger.Types

--------------------------------------------------------------------------------

type State = LensRecord `With` Maybe Observation
                        `With` SearchState
                        `With` TrackState
                        `With` PrDist

defObs ∷ Observation
defObs = Observation { obsPos = Vec2D 0 0
                     , obsCov = mat22 0 0 0 0
                     , obsTime = 0
                     }

initState ∷ IO State
initState = return $ LensRecord `With` Just defObs
                                `With` SearchState Unknown
                                `With` TrackState  Unknown
                                `With` prRedPrior

getObservation ∷ State → Observation
getObservation st = fromMaybe defObs (getL st)

--------------------------------------------------------------------------------

σObs ∷ Double
σObs = 0.01

stereoObservation ∷ Robot → Double → IO (Maybe Observation)
stereoObservation roger time = computeAverageRedPixel roger
  <&> \avgRed → mapPairM (\i → i avgRed <&> imageCoordToAngle <&> (+ i eyeθ))
  <&> \Pair { left = γL, right = γR } →
    let λL   = (2 * baseline) / sq (sin (γR - γL))

        refb = fromList
          [ 2 * baseline * ((cos γR * cos γL) / sin (γR - γL))
          , 2 * baseline * ((cos γR * sin γL) / sin (γR - γL)) + baseline
          , 0
          , 1
          ] ∷ Vec4 Double

        refw = constructwTb basePosition `multmv` refb

        _JB  = mat22 (sq (cos γR))     (-(sq (cos γL)))
                     (sin γR * cos γR) ((-(sin γL)) * cos γL)
               `multms` λL

        wRb  = mat22 (cos baseθ) (-(sin baseθ))
                     (sin baseθ) (cos baseθ)

        _JW  = wRb `multmm` _JB

    in Observation { obsPos  = pack (take n2 refw)
                   , obsCov  = (_JW `multmm` transpose _JW) `multms` sq σObs
                   , obsTime = time
                   }
  where Robot{..} = roger
        baseθ     = θOf basePosition
        sq x      = x * x
        m `multms` s = map (map (* s)) m -- matrix * scalar
        (<&>) ∷ Functor f ⇒ f a → (a → b) → f b
        (<&>) = flip fmap

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st time =
  do st' `With` roger' ← execStateT searchtrack (st `With` roger)
     obs ← stereoObservation roger' time
     return (roger', putL obs st')

enterParams ∷ State → IO State
enterParams = return

