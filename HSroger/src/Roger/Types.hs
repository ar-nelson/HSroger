{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE RankNTypes            #-}
{-# LANGUAGE RecordWildCards       #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.Types( VectorAndAngle(..)
                  , Pair(..)
                  , ArmPair(..)
                  , mapPair
                  , mapPairM
                  , mapArms
                  , Image
                  , RGB
                  , imagePixels
                  , pixelsOf
                  , red
                  , green
                  , blue
                  , Observation(..)
                  , ControlStatus(..)
                  , constructwTb
                  , xOf
                  , yOf
                  , polar
                  , unpolar
                  , mat22
                  , mat44
                  , clampAngle
) where

import           Control.Applicative
import           Control.Monad
import           Data.Vec
import           Data.Word
import           Foreign.C.Types
import           Foreign.Ptr
import           Foreign.Storable

--------------------------------------------------------------------------------

data VectorAndAngle = VectorAndAngle { xyOf ∷ Vec2D
                                     , θOf  ∷ Double
                                     } deriving Show

data Pair α = Pair { left  ∷ α
                   , right ∷ α
                   } deriving Show

data ArmPair α = ArmPair { shoulder ∷ α
                         , elbow    ∷ α
                         } deriving Show

mapPair ∷ ((∀ α. Pair α → α) → β) → Pair β
mapPair fn = Pair (fn left) (fn right)

mapPairM ∷ Monad μ ⇒ ((∀ α. Pair α → α) → μ β) → μ (Pair β)
mapPairM fn = liftM2 Pair (fn left) (fn right)

mapArms ∷ ((∀ α. Pair (ArmPair α) → α) → β) → Pair (ArmPair β)
mapArms fn = Pair (ArmPair (fn (shoulder . left))  (fn (elbow . left)))
                  (ArmPair (fn (shoulder . right)) (fn (elbow . right)))

--------------------------------------------------------------------------------

newtype Image = Image (Ptr Word8)
newtype RGB   = RGB (Ptr Word8)

imagePixels ∷ Int
imagePixels = 128

pixelsOf ∷ Image → [RGB]
pixelsOf (Image ptr) =
  fmap (RGB . (ptr `plusPtr`)) [0, intSize * 3 .. (imagePixels-1) * intSize * 3]

red   ∷ RGB → IO Word8
red   (RGB ptr) = peek ptr

green ∷ RGB → IO Word8
green (RGB ptr) = peekByteOff ptr intSize

blue  ∷ RGB → IO Word8
blue  (RGB ptr) = peekByteOff ptr (2 * intSize)

--------------------------------------------------------------------------------

data ControlStatus = Unknown
                   | NoReference
                   | Transient
                   | Converged
                   deriving (Eq, Ord, Show)

data Observation = Observation { obsPos  ∷ Vec2D
                               , obsCov  ∷ Mat22 Double
                               , obsTime ∷ Double
                               } deriving Show

--------------------------------------------------------------------------------

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

mat22 ∷ α → α → α → α → Mat22 α
mat22 a0 a1 b0 b1 =  (a0 :. a1 :. ())
                  :. (b0 :. b1 :. ())
                  :. ()

mat44 ∷ α → α → α → α
      → α → α → α → α
      → α → α → α → α
      → α → α → α → α → Mat44 α
mat44 a0 a1 a2 a3
      b0 b1 b2 b3
      c0 c1 c2 c3
      d0 d1 d2 d3 =  (a0 :. a1 :. a2 :. a3 :. ())
                  :. (b0 :. b1 :. b2 :. b3 :. ())
                  :. (c0 :. c1 :. c2 :. c3 :. ())
                  :. (d0 :. d1 :. d2 :. d3 :. ())
                  :. ()

constructwTb ∷ VectorAndAngle → Mat44 Double
constructwTb basePos = mat44

  c0  (-s0) 0  (xOf (xyOf basePos))
  s0  c0    0  (yOf (xyOf basePos))
  0   0     1  0
  0   0     0  1

  where s0 = sin (θOf basePos)
        c0 = cos (θOf basePos)

clampAngle ∷ Double → Double
clampAngle θ | θ > pi    = clampAngle (θ - twoPi)
             | θ < -pi   = clampAngle (θ + twoPi)
             | otherwise = θ
             where twoPi = 2 * pi

--------------------------------------------------------------------------------

doubleSize ∷ Int
doubleSize = sizeOf (undefined ∷ CDouble)

intSize ∷ Int
intSize = sizeOf (undefined ∷ CInt)

instance Storable VectorAndAngle where
  sizeOf _ = 3 * doubleSize
  alignment _ = 4
  peek p = liftM2 VectorAndAngle (liftM2 Vec2D (peek p')
                                               (peekElemOff p' 1))
                                 (peekElemOff p' 2)
           where p' = castPtr p ∷ Ptr Double
  poke p (VectorAndAngle (Vec2D x y) θ) =
    poke p' x >> pokeElemOff p' 1 y >> pokeElemOff p' 2 θ
    where p' = castPtr p ∷ Ptr Double

instance Storable (Pair Double) where
  sizeOf _ = 2 * doubleSize
  alignment _ = 4
  peek p = liftM2 Pair (peek p') (peekElemOff p' 1)
           where p' = castPtr p ∷ Ptr Double
  poke p (Pair l r) =  poke p' l >> pokeElemOff p' 1 r
                       where p' = castPtr p ∷ Ptr Double

instance Storable (Pair (ArmPair Double)) where
  sizeOf _ = 4 * doubleSize
  alignment _ = 4
  peek p = Pair <$> (ArmPair <$> get 0 <*> get 1)
                <*> (ArmPair <$> get 2 <*> get 3)
           where get = peekElemOff (castPtr p ∷ Ptr Double)
  poke p (Pair (ArmPair l1 l2) (ArmPair r1 r2)) =
    put 0 l1 >> put 1 l2 >> put 2 r1 >> put 3 r2
    where put = pokeElemOff (castPtr p ∷ Ptr Double)

instance Storable (Pair Vec2D) where
  sizeOf _ = 4 * doubleSize
  alignment _ = 4
  peek p = Pair <$> (Vec2D <$> get 0 <*> get 1)
                <*> (Vec2D <$> get 2 <*> get 3)
           where get = peekElemOff (castPtr p ∷ Ptr Double)
  poke p (Pair (Vec2D x1 y1) (Vec2D x2 y2)) =
    put 0 x1 >> put 1 y1 >> put 2 x2 >> put 3 y2
    where put = pokeElemOff (castPtr p ∷ Ptr Double)

instance Storable (Pair Image) where
  sizeOf _ = 2 * intSize * imagePixels * 3
  alignment _ = 4
  peek p = return $ Pair (Image $ castPtr p)
                         (Image $ castPtr $ p `plusPtr` (intSize*imagePixels*3))
  poke _ _ = return ()

instance Storable Observation where
  sizeOf _ = 7 * doubleSize
  alignment _ = 4
  peek p = Observation <$> (Vec2D <$> get 0 <*> get 1)
                       <*> (mat22 <$> get 2 <*> get 3 <*> get 4 <*> get 5)
                       <*> get 6
           where get = peekElemOff (castPtr p ∷ Ptr Double)
  poke p Observation{..} =
    do put 0 (xOf obsPos)
       put 1 (yOf obsPos)
       put 2 (xOf (xOf obsCov))
       put 3 (yOf (xOf obsCov))
       put 4 (xOf (yOf obsCov))
       put 5 (yOf (yOf obsCov))
       put 6 obsTime
    where put = pokeElemOff (castPtr p ∷ Ptr Double)

