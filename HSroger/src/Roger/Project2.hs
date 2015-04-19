{-# LANGUAGE Rank2Types    #-}
{-# LANGUAGE UnicodeSyntax #-}

module Roger.Project2( State
                     , control
                     , enterParams
                     , reset
                     , initState
                     , fwdArmKinematics
                     , invArmKinematics
) where

import           Control.Arrow ((***))
import           Data.Vec
import           Prelude       hiding (take)
import           Roger.Robot
import           Roger.Types

--------------------------------------------------------------------------------

type State = ()

initState ∷ IO State
initState = return ()

--------------------------------------------------------------------------------

fwdArmKinematics ∷ Robot → (∀ α. Pair α → α) → Vec2D
fwdArmKinematics roger limb = pack (take n2 refb)

  where refb = wT3 `multmv` refw
        refw = 0 :. 0 :. 0 :. 1 :. ()

        wT3  = wT0 `multmm` _0T3
        wT0  = wTb `multmm` bT0
        wTb  = constructwTb (basePosition roger)

        bT0  = mat44 1 0 0 (limb armOffset)
                     0 1 0 0
                     0 0 1 0
                     0 0 0 1

        _0T3 = mat44 c12 (-s12) 0 (l1*c1 + l2*c12)
                     s12 c12    0 (l1*s1 + l2*s12)
                     0   0      1 0
                     0   0      0 1

        c1  = cos (shoulder θ)
        s1  = sin (shoulder θ)
        c12 = cos (shoulder θ + elbow θ)
        s12 = cos (shoulder θ + elbow θ)

        l1 = shoulder armLength
        l2 = elbow armLength
        θ  = limb (armθ roger)


invArmKinematics ∷ Robot → (∀ α. Pair α → α) → Vec2D → Maybe (ArmPair Double)
invArmKinematics roger limb target =
  do bTw ← invert (constructwTb (basePosition roger))
     let refw = xOf target :. (yOf target + limb armOffset) :. 0 :. 1 :. ()
         refb = bTw `multmv` refw
         r2   = sq (xOf refb) + sq (yOf refb)
         c2'  = (r2 - sq l1 - sq l2) / (2 * l1 * l2)

     c2 ← if -1 <= c2' && c2' <= 1 then Just c2' else Nothing
     let s2 = both (* sqrt(1 - sq c2)) (1, -1)
         k1 = l1 + l2 * c2
         k2 = both (* l2) s2
         α  = both (`atan2` k1) k2
         θ  = (`both` (fst, snd)) $ \i →
                ArmPair { shoulder = atan2 (yOf refb) (xOf refb) - i α
                        , elbow    = atan2 (i s2) c2
                        }

     return $ if limb Pair { left  = elbow (fst θ) <= 0
                           , right = elbow (fst θ) >= 0
                           }
                then fst θ else snd θ

  where both f = f *** f
        sq x   = x * x
        l1     = shoulder armLength
        l2     = elbow armLength

--------------------------------------------------------------------------------

control ∷ Robot → State → Double → IO (Robot, State)
control r s _ = return (r, s)

--------------------------------------------------------------------------------

enterParams ∷ State → IO State
enterParams = return

reset ∷ Robot → State → IO (Robot, State)
reset r _ = initState >>= \s → return (r, s)

