{-# LANGUAGE Arrows                #-}
{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Project6( State
                     , control
                     , enterParams
                     , initState
                     , getEstimate
                     , blankEstimate
                     , kalmanFilter
) where

import           Control.Applicative
import           Control.Arrow        hiding (left, right)
import           Control.Monad.Reader
import           Data.Vec
import           Prelude              hiding (take)
import           Roger.Math
import           Roger.Project4       (searchtrack)
import           Roger.Project5       (stereoObservation)
import           Roger.Robot
import           Roger.Types
import           Roger.Wire

type State = (Wire (ReaderT Time IO) Robot (Robot, Estimate), Estimate)

initState :: IO State
initState = return (searchAndFilter, blankEstimate)
  where searchAndFilter = (searchtrack >>^ fst) &&& kalmanFilter

getEstimate ∷ State → Estimate
getEstimate = snd

--------------------------------------------------------------------------------

obsDT  ∷ Double
obsDT  = 0.001

σ_obs  ∷ Double
σ_obs  = 0.01

σ_proc ∷ Double
σ_proc = 10.0

_A ∷ Mat44 Double
_A = mat44 ( 1, 0, obsDT, 0
           , 0, 1, 0    , obsDT
           , 0, 0, 1    , 0
           , 0, 0, 0    , 1
           )

_Qk ∷ Mat44 Double
_Qk = mat44 ( cov4, 0   , cov3, 0
            , 0   , cov4, 0   , cov3
            , cov3, 0   , cov2, 0
            , 0   , cov3, 0   , cov2
            )
    where cov4 = (σ_proc**2) * (obsDT**4)/4
          cov3 = (σ_proc**2) * (obsDT**3)/2
          cov2 = (σ_proc**2) * (obsDT**2)

_H ∷ Mat24 Double
_H = ( 1 :. 0 :. 0 :. 0 :. () )
  :. ( 0 :. 1 :. 0 :. 0 :. () )
  :. ()

_Rk ∷ Mat22 Double
_Rk = identity

initEstimate ∷ Observation → Double → Estimate
initEstimate Observation{..} time = Estimate
  { estState = Vec4D (xOf obsPos) (yOf obsPos) 0 0
  , estCov = mat44 ( xOf (xOf obsCov), yOf (xOf obsCov), 0, 0
                   , xOf (yOf obsCov), yOf (yOf obsCov), 0, 0
                   , 0               , 0               , 1, 0
                   , 0               , 0               , 0, 1
                   )
  , estTime = time
  }

blankEstimate ∷ Estimate
blankEstimate = Estimate { estState = Vec4D 0 0 0 0
                         , estCov   = identity
                         , estTime  = 0
                         }

--------------------------------------------------------------------------------

kalmanFilter ∷ (MonadReader Time m, MonadIO m) ⇒ Wire m Robot Estimate
kalmanFilter = (>>^ snd) $ stateWire blankEstimate $ proc (roger, xPlus) →
  do xMinus ← extrapolate -< xPlus
     xPlus' ← newObservation <|> arr snd -< (roger, xMinus)
     returnA -< ((), reflectOnWall (clampEstimate xPlus'))

  where extrapolate = wire $ \Estimate{..} →
          asks timeSeconds >>= \now → return Estimate
            { estState = pack (_A `multmv` unpack estState)
            , estCov   = _A `multmm` (estCov `multmm` transpose _A) + _Qk
            , estTime  = now
            }

        newObservation = localStateWire False $
          proc ((roger, xMinus), initialized) →
            do obs ← stereoObservation -< roger
               if initialized
                  then do xPlus' ← update -< (roger, obs, xMinus)
                          returnA -< (xPlus', True)
                  else do now ← wire (const (asks timeSeconds)) -< ()
                          returnA -< (initEstimate obs now, True)

        update = maybeWire.wire $ \(Robot{..}, Observation{..}, Estimate{..}) →
          let dist = norm (xyOf basePosition - take n2 estState)
              isoσ_obs = dist * σ_obs * 8
              _R   = _Rk `multms` sq isoσ_obs
              _HT  = transpose _H
              invm = invert (multmm _H (multmm estCov _HT) + _R)
              mKk  = fmap (multmm estCov . multmm _HT) invm
          in asks timeSeconds >>= \now → return $ (\_Kk → Estimate
               { estState = estState + pack (_Kk `multmv`
                              (unpack obsPos - _H `multmv` unpack estState))
               , estCov   = multmm (identity - multmm _Kk _H) estCov
               , estTime  = now
               }) `fmap` mKk

        reflectOnWall = reflectX . reflectY

        reflectX xPlus =
          if (x' < 0 && x <= minX + ballRadius)
          || (x' > 0 && x >= maxX - ballRadius)
             then xPlus { estState = set n2 (-x') (estState xPlus) }
             else xPlus
          where x  = xOf (estState xPlus)
                x' = get n2 (estState xPlus)

        reflectY xPlus =
          if (y' < 0 && y <= minY + ballRadius)
          || (y' > 0 && y >= maxY - ballRadius)
             then xPlus { estState = set n3 (-y') (estState xPlus) }
             else xPlus
          where y  = yOf (estState xPlus)
                y' = get n3 (estState xPlus)

        clampEstimate e = e { estState = clampY (clampX (estState e)) }
          where clampX v = set n0 (max minX (min maxX (get n0 v))) v
                clampY v = set n1 (max minY (min maxY (get n1 v))) v

--------------------------------------------------------------------------------

control :: Robot -> State -> Double -> IO (Robot, State)
control roger (w, _) t = do (mt, w') ← runReaderT (runWire w roger) (Time t)
                            case mt of
                              Just (roger', est) → return (roger', (w', est))
                              Nothing → return (roger, (w', blankEstimate))

enterParams :: State -> IO State
enterParams = return

