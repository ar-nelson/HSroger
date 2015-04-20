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
import           Roger.TypedLens
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

sampleGazeDirection ∷ (Lens PrDist s, MonadState s m, MonadIO m)
                    ⇒ m (Maybe Double)
-- The original function contained a lot of dead code.
-- Presumably it was meant to be improved by the student?
--
-- This implementation takes and returns the distribution in case I want to
-- update it, but it currently returns the distribution unchanged.
sampleGazeDirection =
  do dist ← lgetSt
     rnd  ← liftIO $ randomRIO (0.0, prArea dist)
     let accum i sum | sum < rnd = accum (i + 1) (sum + (prBins dist !! i))
                     | otherwise = i
         bin = fromIntegral (accum 0 0) ∷ Double
     return $ if prArea dist < 0.02
                 then Nothing
                 else Just ((bin+0.5) * prBinSize dist - pi)

