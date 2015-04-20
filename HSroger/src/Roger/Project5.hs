{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE TypeOperators   #-}
{-# LANGUAGE UnicodeSyntax   #-}

module Roger.Project5( State
                     , control
                     , enterParams
                     , reset
                     , initState
                     , Observation(..)
                     , stereoObservation
) where

import           Control.Monad
import           Control.Monad.State hiding (State)
import           Data.Maybe          (maybeToList)
import           Data.Vec
import           Prelude             hiding (map, take)
import           Roger.Project3      (computeAverageRedPixel)
import           Roger.Project4      (SearchState (..), TrackState (..),
                                      searchtrack)
import           Roger.Robot
import           Roger.Sampling      (PrDist, prRedPrior)
import           Roger.TypedLens
import           Roger.Types

--------------------------------------------------------------------------------

type State = Maybe Observation :* SearchState :* TrackState :* PrDist :* ()

initState ∷ IO State
initState = return $ Just Observation { obsPos = 0 :. 0 :. ()
                                      , obsCov = mat22 0 0 0 0
                                      }
                  :* SearchState UNKNOWN
                  :* TrackState  UNKNOWN
                  :* prRedPrior
                  :* ()

--------------------------------------------------------------------------------

data Observation = Observation { obsPos ∷ Vec2 Double
                               , obsCov ∷ Mat22 Double
                               } deriving Show

σobs ∷ Double
σobs = 0.025

stereoObservation ∷ Robot → IO (Maybe Observation)
stereoObservation roger = computeAverageRedPixel roger <&> \avgRed →
  mapPairM (\i → fmap (i eyeθ +) (i avgRed)) <&> \Pair{left = γL, right = γR} →

    let λL   = (2 * baseline) / sq (sin (γR - γL))

        refb = (2 * baseline * ((cos γR * cos γL) / sin (γR - γL)))
            :. (baseline + 2 * baseline * ((cos γR * sin γL) / sin (γR - γL)))
            :. 0
            :. 1 :. ()

        refw = constructwTb basePosition `multmv` refb

        _JB  = mat22 (sq (cos γR))     (-(sq (cos γL)))
                     (sin γR * cos γR) ((-(sin γL)) * cos γL)
               `multms` λL

        wRb  = mat22 (cos baseθ) (-(sin baseθ))
                     (sin baseθ) (cos baseθ)

        _JW  = wRb `multmm` _JB

    in Observation { obsPos = take n2 refw
                   , obsCov = (_JW `multmm` transpose _JW) `multms` σobs
                   }
  where Robot{..} = roger
        baseθ     = θOf basePosition
        sq x      = x * x
        m `multms` s = map (map (* s)) m -- matrix * scalar
        (<&>) ∷ Functor f ⇒ f a → (a → b) → f b
        (<&>) = flip fmap

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ = do roger' :* st' ← execStateT searchtrack (roger :* st)
                        obs ← stereoObservation roger'
                        forM_ (maybeToList obs) print
                        return (roger', lput obs st')

enterParams ∷ State → IO State
enterParams = return

reset ∷ Robot → State → IO (Robot, State)
reset r _ = initState >>= \s → return (r, s)

