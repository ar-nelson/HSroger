{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Math( xOf
                 , yOf
                 , polar
                 , unpolar
                 , clampAngle
                 , vec4
                 , mat22
                 , mat44
                 , rotateMat22
                 , constructwTb
) where

import           Data.Vec
import           Roger.Types

xOf ∷ Access N0 α υ ⇒ υ → α
xOf = get n0

yOf ∷ Access N1 α υ ⇒ υ → α
yOf = get n1

polar ∷ ( Access N0 Double v, Access N1 Double v, Num v, Fold v Double
        , ZipWith Double Double Double v v v
        ) ⇒ v → (Double, Double)
polar v = (norm v, atan2 (yOf v) (xOf v))

unpolar ∷ Double → Double → Vec2D
unpolar r θ = Vec2D (r * cos θ) (r * sin θ)

clampAngle ∷ Double → Double
clampAngle θ | θ > pi    = clampAngle (θ - twoPi)
             | θ < -pi   = clampAngle (θ + twoPi)
             | otherwise = θ
             where twoPi = 2 * pi

vec4 ∷ (α, α, α, α) → Vec4 α
vec4 (x, y, z, w) = x :. y :. z :. w :. ()

mat22 ∷ (α, α, α, α) → Mat22 α
mat22 (a0, a1, b0, b1) =  (a0 :. a1 :. ())
                       :. (b0 :. b1 :. ())
                       :. ()

mat44 ∷ ( α, α, α, α
        , α, α, α, α
        , α, α, α, α
        , α, α, α, α ) → Mat44 α
mat44 ( a0, a1, a2, a3
      , b0, b1, b2, b3
      , c0, c1, c2, c3
      , d0, d1, d2, d3 ) = (a0 :. a1 :. a2 :. a3 :. ())
                        :. (b0 :. b1 :. b2 :. b3 :. ())
                        :. (c0 :. c1 :. c2 :. c3 :. ())
                        :. (d0 :. d1 :. d2 :. d3 :. ())
                        :. ()

rotateMat22 ∷ RealFloat α ⇒ α → Mat22 α
rotateMat22 θ = mat22 ( cos θ, -(sin θ)
                      , sin θ, cos θ
                      )

constructwTb ∷ VectorAndAngle → Mat44 Double
constructwTb basePos = mat44 ( c0,  -s0, 0,  xOf (xyOf basePos)
                             , s0,  c0 , 0,  yOf (xyOf basePos)
                             , 0 ,  0  , 1,  0
                             , 0 ,  0  , 0,  1
                             )
                     where s0 = sin (θOf basePos)
                           c0 = cos (θOf basePos)

