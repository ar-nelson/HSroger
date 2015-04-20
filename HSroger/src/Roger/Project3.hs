{-# LANGUAGE UnicodeSyntax #-}

module Roger.Project3( State
                     , control
                     , enterParams
                     , reset
                     , initState
                     , computeAverageRedPixel
                     , imageCoordToAngle
) where

import           Control.Monad
import           Roger.Robot
import           Roger.Types

type State = ()

initState ∷ IO State
initState = return ()

--------------------------------------------------------------------------------

computeAverageRedPixel ∷ Robot → IO (Pair (Maybe Double))
computeAverageRedPixel roger =

  mapPairM $ \eye →
    liftM avg (foldM accum (0, 0.0) (pixelsOf (eye (image roger)) `zip` [0..]))

  where avg (redFound, coordsSum) =
          if redFound > 0 then Just (coordsSum / redFound)
                          else Nothing
        accum (redFound, coordsSum) (rgb, x) =
          do r <- red rgb
             g <- green rgb
             b <- blue rgb
             if r > g && r > b then return (redFound + 1, coordsSum + x)
                               else return (redFound, coordsSum)

imageCoordToAngle ∷ Double → Double
imageCoordToAngle imageCoord =
  (imageCoord - focalLength) / focalLength * (pi / 4.0)

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control roger st _ =
  computeAverageRedPixel roger >>= \avgRed →
    let θerror eye = case eye avgRed of
                       Just a  → -(imageCoordToAngle a)
                       Nothing → 0.0
        roger' = roger {eyeSetpoint = mapPair (\i → i (eyeθ roger) - θerror i)}
    in return (roger', st)

enterParams ∷ State → IO State
enterParams = return

reset ∷ Robot → State → IO (Robot, State)
reset r s = return (r, s)

