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
                     , SearchState(..)
                     , TrackState(..)
                     , search
                     , track
                     , searchtrack
                     ) where

import           Control.Monad.State hiding (State)
import           Data.Maybe          (isJust)
import           Roger.Math
import           Roger.Project3      (computeAverageRedPixel, imageCoordToAngle)
import           Roger.Robot
import           Roger.Sampling
import           Roger.TypedLens
import           Roger.Types

newtype SearchState = SearchState { getSearchState ∷ ControlStatus }
newtype TrackState  = TrackState  { getTrackState ∷ ControlStatus }

instance Show SearchState where show (SearchState s) = "SearchState " ++ show s
instance Show TrackState where show (TrackState s) = "TrackState " ++ show s

type State = LensRecord `With` SearchState
                        `With` TrackState
                        `With` PrDist

initState ∷ IO State
initState = return $ LensRecord `With` SearchState Unknown
                                `With` TrackState  Unknown
                                `With` prRedPrior

--------------------------------------------------------------------------------

ε ∷ Double
ε = 0.01

search ∷ ( Has Robot s, Has SearchState s, Has PrDist s
         , MonadState s m, MonadIO m
         ) ⇒ m ()
search = getsStateL getSearchState >>= doSearch
  where doSearch Converged = return ()
        doSearch Transient =
          do Robot{..} ← getStateL
             let heading   = θOf baseSetpoint
                 baseθ     = θOf basePosition
                 eyeTarget = clampAngle (heading - baseθ)
             setRoger's EyeSetpoint (mapPair (const eyeTarget))
             setDebugL SearchState $ if abs (baseθ - heading) < ε
                                        then Converged
                                        else Transient
        doSearch _ = maybe (setDebugL SearchState NoReference) setHeading
                       =<< sampleGazeDirection
        setHeading heading =
          do roger ← getStateL
             setRoger's BaseSetpoint (baseSetpoint roger) { θOf = heading }
             doSearch Transient

track ∷ (Has Robot s, Has TrackState s, MonadState s m, MonadIO m) ⇒ m ()
track = do roger@Robot{..} ← getStateL
           avgRed ← liftIO (computeAverageRedPixel roger)
           let eye ∷ (∀ α. Pair α → α) → Double
               eye i = i eyeθ - θError
                 where θError = maybe 0.0 (negate . imageCoordToAngle) (i avgRed)
               avgBaseError = (0 - left eyeθ - right eyeθ) / 2.0
               base = baseSetpoint
                 { θOf = clampAngle (θOf basePosition - avgBaseError) }

           if isJust (left avgRed) || isJust (right avgRed)
              then setRoger's EyeSetpoint (mapPair eye) >>
                   if abs avgBaseError < ε
                      then setDebugL TrackState Converged
                      else setDebugL TrackState Transient >>
                           setRoger's BaseSetpoint base
              else TrackState *= NoReference

searchtrack ∷ ( Has Robot s, Has SearchState s, Has TrackState s , Has PrDist s
              , MonadState s m, MonadIO m
              ) ⇒ m ()
searchtrack = get >>= \st →
  case st *. getSearchState of
    Unknown   → case st *. getTrackState of
      NoReference → msg "SEARCH" >> search
      _           → track
    Converged → msg "TRACK" >> setDebugL SearchState Unknown >> track
    _         → search
  where msg s = liftIO (putStrLn ("Switched to " ++ s ++ " mode."))

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ =
  do st' `With` roger' ← execStateT searchtrack (st `With` roger)
     return (roger', st')

enterParams ∷ State → IO State
enterParams = return

