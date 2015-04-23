{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Sampling( PrDist(..)
                     , distBins
                     , prRedPrior
                     , sampleGazeDirection
) where

import           Control.Monad.State
import           Prelude             hiding (sum)
import           Roger.Wire
import           System.Random

data PrDist = PrDist { prBinSize ∷ Double
                     , prArea    ∷ Double
                     , prBins    ∷ [Double]
                     }

distBins ∷ Int
distBins = 64

nHeadings ∷ Double
nHeadings = 64

prRedPrior ∷ PrDist
prRedPrior = PrDist { prBinSize = (2.0 * pi) / nHeadings
                    , prArea    = 1.0
                    , prBins    = replicate distBins (1.0/nHeadings)
                    }

sampleGazeDirection ∷ (MonadIO m) ⇒ PrDist → Wire m a Double
-- The original function contained a lot of dead code.
-- Presumably it was meant to be improved by the student?
--
-- This implementation treats the distribution as mutable state, but currently
-- leaves it unchanged.
sampleGazeDirection = stateWire (maybeWire (wire fn))
  where fn _ = do dist ← get
                  rnd  ← liftIO $ randomRIO (0.0, prArea dist)
                  let accum i sum
                        | sum < rnd = accum (i + 1) (sum + (prBins dist !! i))
                        | otherwise = i
                      bin = fromIntegral (accum 0 0) ∷ Double
                  return $ if prArea dist < 0.02
                              then Nothing
                              else Just ((bin+0.5) * prBinSize dist - pi)

