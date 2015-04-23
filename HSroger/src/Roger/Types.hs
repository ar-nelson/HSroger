{-# LANGUAGE FlexibleInstances #-}
{-# LANGUAGE RankNTypes        #-}
{-# LANGUAGE RecordWildCards   #-}
{-# LANGUAGE UnicodeSyntax     #-}

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
                  , ControlStatus(..)
                  , Observation(..)
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
  peek p = Pair <$> (ArmPair <$> peekAt 0 <*> peekAt 1)
                <*> (ArmPair <$> peekAt 2 <*> peekAt 3)
           where peekAt = peekElemOff (castPtr p ∷ Ptr Double)
  poke p (Pair (ArmPair l1 l2) (ArmPair r1 r2)) =
    pokeAt 0 l1 >> pokeAt 1 l2 >> pokeAt 2 r1 >> pokeAt 3 r2
    where pokeAt = pokeElemOff (castPtr p ∷ Ptr Double)

instance Storable (Pair Vec2D) where
  sizeOf _ = 4 * doubleSize
  alignment _ = 4
  peek p = Pair <$> (Vec2D <$> peekAt 0 <*> peekAt 1)
                <*> (Vec2D <$> peekAt 2 <*> peekAt 3)
           where peekAt = peekElemOff (castPtr p ∷ Ptr Double)
  poke p (Pair (Vec2D x1 y1) (Vec2D x2 y2)) =
    pokeAt 0 x1 >> pokeAt 1 y1 >> pokeAt 2 x2 >> pokeAt 3 y2
    where pokeAt = pokeElemOff (castPtr p ∷ Ptr Double)

instance Storable (Pair Image) where
  sizeOf _ = 2 * intSize * imagePixels * 3
  alignment _ = 4
  peek p = return $ Pair (Image $ castPtr p)
                         (Image $ castPtr $ p `plusPtr` (intSize*imagePixels*3))
  poke _ _ = return ()

instance Storable Observation where
  sizeOf _ = 7 * doubleSize
  alignment _ = 4
  peek p = Observation <$> (Vec2D <$> peekAt 0 <*> peekAt 1)
                       <*> do a ← peekAt 2
                              b ← peekAt 3
                              c ← peekAt 4
                              d ← peekAt 5
                              return $ (a :. b :. ()) :.
                                       (c :. d :. ()) :. ()
                       <*> peekAt 6
           where peekAt = peekElemOff (castPtr p ∷ Ptr Double)
  poke p Observation{..} =
    do pokeAt 0 (get n0 obsPos)
       pokeAt 1 (get n1 obsPos)
       pokeAt 2 (get n0 (get n0 obsCov))
       pokeAt 3 (get n1 (get n0 obsCov))
       pokeAt 4 (get n0 (get n1 obsCov))
       pokeAt 5 (get n1 (get n1 obsCov))
       pokeAt 6 obsTime
    where pokeAt = pokeElemOff (castPtr p ∷ Ptr Double)

