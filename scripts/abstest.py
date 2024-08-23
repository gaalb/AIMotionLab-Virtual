from abc import ABC, abstractmethod

class Base(ABC):
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        print(f"Initializing subclass of Base: {cls.__name__}", kwargs)

class AbstractSub(Base):
    @abstractmethod
    def an_abstract_method(self):
        pass

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        print(f"Initializing subclass of AbstractSub: {cls.__name__}", kwargs)

class Sub(AbstractSub):
    def an_abstract_method(self):
        print("Implementing the abstract method in Sub.")

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        print(f"Initializing subclass of Sub: {cls.__name__}", kwargs)

# Example of creating a concrete subclass of Sub, passing arguments to __init_subclass__
class ConcreteSub(Sub, msg="Hello, ConcreteSub!"):
    pass
