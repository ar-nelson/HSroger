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

search ∷ (MonadIO m) ⇒ Wire (StateT ControlStatus m) Robot Robot
search = (stateIs Converged >>> skip (wire $ const $
             put Unknown >> liftIO (putStrLn "Restarting search...")))
     <|> (stateIs Transient >>> waitToAlign)
     <|> (id &&& sampleGazeDirection prRedPrior >>> setHeading)

  where stateIs s = skip (wire (const get) >>> ifWire (== s))

        waitToAlign = wire $ \roger@Robot{..} →
          let heading   = θOf baseSetpoint
              baseθ     = θOf basePosition
              eyeTarget = clampAngle (heading - baseθ)
              nextState = if abs (baseθ - heading) < ε then Converged
                                                       else Transient
          in do put nextState
                return (roger { eyeSetpoint = mapPair (const eyeTarget) })

        setHeading = arr setBaseSetpoint ^>> waitToAlign

        setBaseSetpoint (roger, heading) =
          roger { baseSetpoint = (baseSetpoint roger) { θOf = heading } }

track ∷ (MonadIO m) ⇒ Wire m Robot (Robot, ControlStatus)
track = avgRedWire &&& id >>^ \(avgRed, roger@Robot{..}) →

  let eye ∷ (∀ α. Pair α → α) → Double
      eye i = i eyeθ - θErr where θErr = -(imageCoordToAngle (i avgRed))
      avgBaseError = (-(left eyeθ) - right eyeθ) / 2.0
      base = baseSetpoint { θOf = clampAngle (θOf basePosition - avgBaseError) }

  in if abs avgBaseError < ε
        then (roger { eyeSetpoint  = mapPair eye }, Converged)
        else (roger { eyeSetpoint  = mapPair eye
                    , baseSetpoint = base
                    }, Transient)

searchtrack ∷ (MonadIO m) ⇒ Wire m Robot (Robot, ControlStatus)
searchtrack = (`stateWire` Unknown) $
  (skip searchConverged >>> track) <|> (search &&& searchState)
  where searchState     = wire (const get)
        searchConverged = searchState >>> ifWire (== Converged)

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ = do (roger', st') ← runWire st roger
                        return (fromMaybe roger roger', st')

enterParams ∷ State → IO State
enterParams = return

