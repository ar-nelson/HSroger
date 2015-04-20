{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE OverlappingInstances  #-}
{-# LANGUAGE TypeOperators         #-}
{-# LANGUAGE UnicodeSyntax         #-}

module Roger.TypedLens( Lens(..)
                      , (*.)
                      , (*=)
                      , lgetSt
                      , lgetsSt
                      , lputSt
                      , LensRecord(..)
                      , With(..)
) where

--------------------------------------------------------------------------------
-- TYPED LENSES
--
-- The "lens pattern" is a commonly-used Haskell design pattern for extensible
-- records. However, the lens library on Hackage is huge, unwieldy*, and
-- difficult to compile in my current Makefile+GHC setup.
--
-- Therefore, I've invented my own extremely-simple substitute. These typed
-- lenses are effectively HLists, which use newtypes as record field names. Lens
-- is a typeclass, not a type, and a value of class `Lens Foo` has a field of
-- type Foo.
--
-- Simple lenses are "snoc lists" (reverse cons lists), and can be constructed
-- using the `With` type operator; the first element of a `With` chain should be
-- the LensRecord type/constructor, which is equivalent to ().
--
-- * https://ro-che.info/articles/2014-04-24-lens-unidiomatic

import           Control.Monad.State

class Lens τ α where
  lget ∷ α → τ
  lput ∷ τ → α → α

(*.) ∷ (Lens τ λ) ⇒ λ → (τ → τ') → τ'
lens *. getter = getter (lget lens)

(*=) ∷ (Lens τ λ, MonadState λ μ) ⇒ (α → τ) → α → μ ()
constructor *= value = lputSt (constructor value)

lgetSt ∷ (Lens τ λ, MonadState λ μ) ⇒ μ τ
lgetSt = gets lget

lgetsSt ∷ (Lens τ λ, MonadState λ μ) ⇒ (τ → α) → μ α
lgetsSt f = gets (*. f)

lputSt ∷ (Lens τ λ, MonadState λ μ) ⇒ τ → μ ()
lputSt = modify . lput

instance Lens τ τ where
  lget x = x
  lput x _ = x

--------------------------------------------------------------------------------

infixl 2 `With`

data LensRecord = LensRecord

data τHead `With` τTail = τHead `With` τTail

instance Lens τ (α `With` τ) where
  lget (_ `With` h) = h
  lput h (t `With` _) = t `With` h

instance Lens τ β ⇒ Lens τ (β `With` α) where
  lget (t `With` _) = lget t
  lput x (t `With` h) = lput x t `With` h

