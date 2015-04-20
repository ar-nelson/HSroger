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
                      , (:*)(..)
) where

--------------------------------------------------------------------------------
-- TYPED LENSES
--
-- The "lens pattern" is a commonly-used Haskell design pattern for extensible
-- records. However, the lens library on Hackage is huge*, unwieldy, and
-- difficult to compile in my current Makefile+GHC setup.
--
-- Therefore, I've invented my own extremely-simple substitute. These typed
-- lenses are effectively HLists, which use newtypes as record field names. Lens
-- is a typeclass, not a type, and an object of class `Lens Foo` has a field of
-- type Foo.
--
-- Simple lenses can be constructed using the :* cons operator.
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

--------------------------------------------------------------------------------

infixr :*

data τHead :* τTail = τHead :* τTail

instance Lens τ (τ :* α) where
  lget (h :* _) = h
  lput h (_ :* t) = h :* t

instance Lens τ β ⇒ Lens τ (α :* β) where
  lget (_ :* t) = lget t
  lput x (h :* t) = h :* lput x t

