struct Ignore<P, O> {
    parser: P,
    _marker: std::marker::PhantomData<fn() -> O>,
}

struct AndIgnore<P, F, O> {
    parser: P,
    f: F,
    _marker: std::marker::PhantomData<fn() -> O>,
}

impl<I, O1, O2, E, F, G> nom::Parser<I, O2, E> for AndIgnore<F, G, O1>
where
    F: nom::Parser<I, O1, E>,
    G: nom::Parser<I, O2, E>,
{
    fn parse(&mut self, input: I) -> nom::IResult<I, O2, E> {
        let (input, _) = self.parser.parse(input);
        self.f.parse(input)
    }
}

impl<P, I, O, E> nom::Parser<I, (), E> for Ignore<P, O>
where
    P: nom::Parser<I, O, E>
{
    fn parse(&mut self, input: I) -> nom::IResult<I, O, E> {
        let (input, _) = self.parser.parse(input)?;
        Ok((input, ()))
    }
}

trait ParserExt<I, O, E>: nom::Parser<I, O, E> {
    fn ignore<G, O2>(self, g: G) -> Ignore<Self>
    where
        G: nom::Parser<I, O2, E>
    {
        Ignore { parser: g }
    }

    fn and_<G, O2>(self, g: G) -> AndIgnore<Self, G>
    where
        G: nom::Parser<O, O2, E>,
        Self: Sized,
    {
        AndIgnore { parser: self, f: g }
    }
}

impl<T, I, O, E> ParserExt<I, O, E> for T
where
    T: nom::Parser<I, O, E>,
{ }
