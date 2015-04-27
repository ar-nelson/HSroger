{-# LANGUAGE Arrows                #-}
{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RankNTypes            #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project4( State
                     , control
                     , enterParams
                     , initState
                     , search
                     , track
                     , searchtrack
                     ) where

import           Control.Applicative
import           Control.Arrow       hiding (left, right)
import           Control.Category
import           Control.Monad.State hiding (State)
import           Data.Maybe          (fromMaybe)
import           Prelude             hiding (id, (.))
import           Roger.Math
import           Roger.Project3      (avgRedWire, imageCoordToAngle)
import           Roger.Robot
import           Roger.Sampling
import           Roger.Types
import           Roger.Wire

type State = Wire IO Robot Robot

initState ∷ IO State
initState = return (searchtrack >>^ fst)

--------------------------------------------------------------------------------

ε ∷ Double
ε = 0.01

search ∷ (MonadIO m) ⇒ Wire m (Robot, ControlStatus) (Robot, ControlStatus)
search = proc (roger, st) → case st of
  Converged → do debugStr -< "Restarting search..."
                 returnA  -< (roger, Unknown)
  Transient → waitToAlign -< roger
  _         → do heading ← sampleGazeDirection prRedPrior -< ()
                 roger'  ← setHeading -< (roger, heading)
                 waitToAlign -< roger'

  where waitToAlign = wire (return . waitToAlign')
        setHeading  = arr (uncurry setHeading')

        waitToAlign' roger@Robot{..} =
          (roger { eyeSetpoint = mapPair (const eyeTarget) }, nextState)
          where heading   = θOf baseSetpoint
                baseθ     = θOf basePosition
                eyeTarget = clampAngle (heading - baseθ)
                nextState = if abs (baseθ - heading) < ε then Converged
                                                         else Transient
        setHeading' roger heading =
          roger { baseSetpoint = (baseSetpoint roger) { θOf = heading } }

track ∷ (MonadIO m) ⇒ Wire m Robot (Robot, ControlStatus)
track = proc roger@Robot{..} →
  do avgRed ← avgRedWire -< roger
     let avgBaseError = (-(left eyeθ) - right eyeθ) / 2.0
         eyes = mapPair (\i → i eyeθ + imageCoordToAngle (i avgRed))
         base = baseSetpoint
           { θOf = clampAngle (θOf basePosition - avgBaseError) }
     returnA -< if abs avgBaseError < ε
                   then (roger { eyeSetpoint  = eyes }, Converged)
                   else (roger { eyeSetpoint  = eyes
                               , baseSetpoint = base
                               }, Transient)

searchtrack ∷ (MonadIO m) ⇒ Wire m Robot (Robot, ControlStatus)
searchtrack = localStateWire Unknown $ proc (roger, searchState) →
  case searchState of
    Converged → do result ← track <|> (id &&& pure NoReference) -< roger
                   returnA -< ( result
                              , case snd result of NoReference → NoReference
                                                   _           → Converged
                              )
    _         → do result ← search <|> id -< (roger, searchState)
                   returnA -< (result, snd result)

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ = do (roger', st') ← runWire st roger
                        return (fromMaybe roger roger', st')

enterParams ∷ State → IO State
enterParams = return

