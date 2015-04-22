{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RankNTypes            #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project4( State
                     , control
                     , enterParams
                     , reset
                     , initState
                     , SearchState(..)
                     , TrackState(..)
                     , search
                     , track
                     , searchtrack
                     ) where

import           Control.Monad.State hiding (State)
import           Data.Maybe          (isJust)
import           Roger.Project3      (computeAverageRedPixel, imageCoordToAngle)
import           Roger.Robot
import           Roger.Sampling
import           Roger.TypedLens
import           Roger.Types

newtype SearchState = SearchState { getSearchState ∷ ControlStatus }
newtype TrackState  = TrackState  { getTrackState ∷ ControlStatus }

type State = LensRecord `With` SearchState
                        `With` TrackState
                        `With` PrDist

initState ∷ IO State
initState = return $ LensRecord `With` SearchState UNKNOWN
                                `With` TrackState  UNKNOWN
                                `With` prRedPrior

--------------------------------------------------------------------------------

ε ∷ Double
ε = 0.01

search ∷ ( Has Robot s, Has SearchState s, Has PrDist s
         , MonadState s m, MonadIO m
         ) ⇒ m ()
search = getsStateL getSearchState >>= doSearch
  where doSearch CONVERGED = return ()
        doSearch TRANSIENT =
          do roger ← getStateL
             let heading   = θOf (baseSetpoint roger)
                 baseθ     = θOf (basePosition roger)
                 eyeTarget = clampAngle (heading - baseθ)
             putStateL roger { eyeSetpoint = mapPair (const eyeTarget) }
             SearchState *= if abs (baseθ - heading) < ε
                               then CONVERGED
                               else TRANSIENT
        doSearch _ = maybe (SearchState *= NO_REFERENCE) setHeading
                       =<< sampleGazeDirection
        setHeading heading =
          do mapStateL (\r → r {baseSetpoint = (baseSetpoint r){θOf = heading}})
             doSearch TRANSIENT

track ∷ (Has Robot s, Has TrackState s, MonadState s m, MonadIO m) ⇒ m ()
track = do roger  ← getStateL
           let Robot{..} = roger
           avgRed ← liftIO (computeAverageRedPixel roger)
           let eye ∷ (∀ α. Pair α → α) → Double
               eye i = i eyeθ - θError
                 where θError = maybe 0.0 (negate . imageCoordToAngle) (i avgRed)
               avgBaseError = (0 - left eyeθ - right eyeθ) / 2.0
               eyes = mapPair eye
               base = baseSetpoint
                 { θOf = clampAngle (θOf basePosition - avgBaseError) }

           if isJust (left avgRed) || isJust (right avgRed)
              then if abs avgBaseError < ε
                      then do TrackState *= CONVERGED
                              mapStateL $ \r → r { eyeSetpoint = eyes }
                      else do TrackState *= TRANSIENT
                              mapStateL $ \r → r { eyeSetpoint  = eyes
                                                 , baseSetpoint = base
                                                 }
              else TrackState *= NO_REFERENCE

searchtrack ∷ ( Has Robot s, Has SearchState s, Has TrackState s , Has PrDist s
              , MonadState s m, MonadIO m
              ) ⇒ m ()
searchtrack = get >>= \st →
  case st *. getSearchState of
    UNKNOWN   → case st *. getTrackState of
                    NO_REFERENCE → msg "SEARCH" >> search
                    _            → track
    CONVERGED → msg "TRACK" >> SearchState *= UNKNOWN >> track
    _         → search
  where msg s = liftIO (putStrLn ("Switched to " ++ s ++ " mode."))

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ =
  do st' `With` roger' ← execStateT searchtrack (st `With` roger)
     return (roger', st')

enterParams ∷ State → IO State
enterParams = return

reset ∷ Robot → State → IO (Robot, State)
reset r _ = initState >>= \s → return (r, s)

