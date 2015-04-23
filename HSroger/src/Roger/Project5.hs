{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project5( State
                     , control
                     , enterParams
                     , initState
                     , stereoObservation
                     , getObservation
) where

import           Control.Arrow
import           Control.Category
import           Control.Monad
import           Control.Monad.Reader
import           Data.Maybe           (fromMaybe)
import           Data.Vec
import           Prelude              hiding (id, map, take, (.))
import           Roger.Math
import           Roger.Project3       (avgRedWire, imageCoordToAngle)
import           Roger.Project4       (searchtrack)
import           Roger.Robot
import           Roger.Types
import           Roger.Wire

--------------------------------------------------------------------------------

type State = (Maybe Observation, Wire IO Robot Robot)

defObs ∷ Observation
defObs = Observation { obsPos = Vec2D 0 0
                     , obsCov = mat22 (0, 0, 0, 0)
                     , obsTime = 0
                     }

initState ∷ IO State
initState = return (Nothing, searchtrack >>^ fst)

getObservation ∷ State → Observation
getObservation st = fromMaybe defObs (fst st)

--------------------------------------------------------------------------------

σObs ∷ Double
σObs = 0.01

stereoObservation ∷ (MonadReader Time m, MonadIO m) ⇒ Wire m Robot Observation
stereoObservation = (avgRedWire &&& id >>>) $ wire $

  \(avgRed, Robot{..}) →

    let Pair γL γR = mapPair (\i → i eyeθ + imageCoordToAngle (i avgRed))
        λL         = (2 * baseline) / sq (sin (γR - γL))

        refb = vec4
          ( 2 * baseline * ((cos γR * cos γL) / sin (γR - γL))
          , 2 * baseline * ((cos γR * sin γL) / sin (γR - γL)) + baseline
          , 0
          , 1
          )
        refw = constructwTb basePosition `multmv` refb

        _JB  = mat22 ( sq (cos γR),     -(sq (cos γL))
                     , sin γR * cos γR, -(sin γL * cos γL)
                     ) `multms` λL

        _JW  = rotateMat22 (θOf basePosition) `multmm` _JB

        obsPos  = pack (take n2 refw)
        obsCov  = (_JW `multmm` transpose _JW) `multms` sq σObs

    in liftM (\obsTime → Observation {..}) (asks timeSeconds)

  where sq x = x * x
        m `multms` s = map (map (* s)) m -- matrix * scalar

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger (_, w) time =
  do (roger', w') ← runWire w roger
     obs'         ← runReaderT (evalWire stereoObservation roger) (Time time)
     return (fromMaybe roger roger', (obs', w'))

enterParams ∷ State → IO State
enterParams = return

