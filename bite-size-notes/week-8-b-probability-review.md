
# Probability Review
## Scope and Objective
In this section we will cover the basic concepts of probability and its relationship to state estimation and controls. Mainly, the topics reviewed will include: 
- Probabilistic Graphical Models
- Random variables
- Probability space
- Mean and variance
- Probability distributions
- Marginalization
- Conditional distributions/probability
- Chain rule
- Bayes rule
- Independence
- Recursive Bayesian Estimator

## Introduction
Probability is a subject which is widely used in industries from manufacturing optimization to robotics control systems. It is a versatile tool to understand and requires a basic fundamental working knowledge of the theory and notation. Generally in controls we model systems as deterministic systems, meaning that given a control input $u$, we will know the next state of the system perfectly (assuming no model errors). However, this is not the case with some dynamical systems, especially ones which exhibit some stochasticity in the states or state measurements. 

For example, if you have an autonomous underwater vehicle and wish to control the position of it in its environment, you will need to know information about where it is relaive to some reference frame. Getting this information might seems straightforward, we can put a GPS module on it and get relatively accurate coordinates in the world coordinate frame. The issue arises due to the nature of the GPS measurements, since they're measurements, how do you know if the measurement you receieved should be trusted? Is it reasonable to be 50 meters away from the point you were just at 2 seconds ago? It's likely not, but how do we characterize and measure the uncertainty of these measurements? These are important considerations to make when designing real world systems, since simulations most always assume a perfect model with deterministic inputs.


## Preliminaries
To start our probability review chapter, we'll first define some important terms and concepts which are crucial for understanding later topics like the recursive bayesian estimator.

**Random Variable**: We define a random variable (or RV) $X$ as a quantity whose value is unknown or uncertain. This is usually denoted with a capital letter in probability literature. Next, we define $x$ as a possible value of $X$ or a sample of $X$, similarly, this is usually denoted with the lowercase version of the random variable.

**Probability Space**: Similarly to vector spaces we have probability space, which represents various sets relating to the random variable.
1. $\Omega$ is defined as the sample space (all the possible outcomes of the RV)
2. $F$ is defined as the event space (all the possible subsets of the set $\Omega$)
3. $P$ is defined as the probability measure (an operator assigning a probability to each event)

**Common Notation**: Usually we will write $P(x)$ as the probability, a measure of chance between 0 and 1, of the random variable $X$ being $x$. So if we have the set $X$ representing a coin toss where the outcome can be heads or tails, we would denote $P(heads) = P(tails) = 0.5$. Note that the probability measure for each possible outcome considering a RV needs to sum to 1: $\sum P(x) = 1$. Additionally, we can have a vector of RVs, such as $\boldsymbol{X} = [X_1, X_2,...,X_n]$, this naturally gives us a multi-dimensional probability function $P(\boldsymbol{x}) = P(x_1, x_2,...,x_n)$. We can similarly have multiple random variables giving rise to multivariate probability functions. Given two RVs $X$ and $Y$, we can define $P(x,y) ~ (X,Y)$ and combining the two to get a multivariate, multi-dimensional probability space, we get $[(X_1, X_2,...,X_n), (Y_1, Y_2,...,Y_n)] ~ P(x_1, x_2,...,x_n,y_1,y_2,...,y_m)$. Since these probability operators can start to get quite long, bold symbols are usually used to denote vectors be sure to take note of the dimensionality of the systems being analyzed.

**Probability Distributions**: Probability distributions describe the likelihood of picking a specific random value based on a probability density function (PDF). Where $X ~ P(x)$, we can describe the expected value (EV) of a probability function as $E(g(x)) = \int_x g(x) P(x) dx$. Here, $g(x) denotes a general function of the variable $x$, and $P(x)$ is the probability measure or distribution function. Note that probability distributions can take on either continuous or discrete forms, so the discrete version of a PDF is called a probability mass function (PMF). In discrete form, the expected value function $E(g(x)) = \sum_x g(x)P(x)$. 

**Mean and Covariance**: Now that we have defined the expected value, we can understand the meaning behind the mean and variance (or covariance) of a probability distribution. The mean $\mu_x = E(x)$ where the function $g(x) = x$ this essentially gives us a weighted average of all of the possible outcomes of the RV. The variance, denoted as $\Sigma_x = E((x-\mu_x)(x-\mu_x)^T)$, represents the spread of possible values from the probability distribution. We can easily visualize the mean and variance in a normal distribution below.

**Normal Distribution**: The normal distribution is the most known and commonly used distribution in probability theory. It is characterized by the bell-shaped curve which leads to symmetric tails which extend to $- \inf$ and $+ \inf$. The equation for the normal distribution is 
$$N(\mu, \Sigma) = \frac{1}{(2 \pi)^{k/2} det(\Sigma)^{1/2}} \exp(-\frac{1}{2}(x-\mu)^T \Sigma^{-1}(x-\mu))$$
Note the parameters include $k$, $\mu$, and $\Sigma$, so once we know the number of dimensions of our data, we can choose the mean and variance accordingly.

**Marginalization**: So far, we've written $(X,Y) ~ P(x,y)$ but haven't given an explicit definition for what two RVs represent in a probability function. In this case, if $X$ and $Y$ were both one-dimensional, then the notation $P(x,y)$ is the probability of the RVs being $x$ **and** $y$. In general, commas separating variables in the probability function represent *and* statements. Marginalization what we do when we want to convert a probability function involving multiple RVs into one involving only a single RV. So if we had $P(x,y)$ and desired either $P(x)$ or $P(y)$, we could marginalize on $P(x,y)$ by integrating the undesired variable out. So $P(x) = \int_y P(x,y) dy$ or in the discrete case $\sum_y P(x,y)$. 

**Conditional Probability**: Sometimes we have RVs which can depend on other RVs, lets say we wanted to model the weather and set the possible states to either cloudy, rainy, or sunny. Maybe we wanted to know what the probability of a rainy day is tomorrow based on the weather today, this would naturally lead into conditonal probability which descrives the probability of getting a specific state (or value) given another state (in this case the weather in the previous day). So the notation $P(x \mid y)$ denotes the probability of getting the value $x$, given the value $y$. If the state today was sunny and we knew based on historical data that the probability of tomorrow being rainy was $0.1$, then we could say $P(rainy \mid sunny) = 0.1$. One important equation for conditional probability is 
$$P(x \mid y) = \frac{P(x,y)}{P(y)}$$
This states that the conditional probability of $x$ given $y$ is a function of the joint probability and probability of $y$ alone.

**Chain Rule**: The chain rule allows us to factorize joint probability distributions into conditional probabilities and those of a single RV. For example: $P(x,y) = P(x \mid y) P(y) = P(y \mid x) P(x)$. Note that there are two seperate but valid ways to factorize the joint distribution. This also works for joint distributions of more than two RVs:
$$P(x,y,z) = P(x \mid y,z) P(y,z) = P(x \mid y,z) P(y \mid z) P(z)$$

**Bayes Rule**: Bayes rule allows us to write the conditional probability of one variable given another based on the reverse conditional probability and probability functions of the individual RVs. The general form of Bayes rule is: 
$$P(x \mid y) = \frac{P(y \mid x) P(x)}{P(y)}$$
Where $P(y) = \int_x P(y \mid x) P(x) dx$ and can be considered a sort of scaling or normalizing function. In practice, $P(y)$ can often be intractable or too computationally expensive to compute so approximations methods can be used. Conceptually, we can describe the terms in Bayes rule as follows: $P(x \mid y) is the posterior, so if $y$ is a measurement of $x$, then what is $x$ given the measurement? $P(x)$ is the prior, which describes the belief or guess of $x$ before the measurement is known or aquired. $P(y \mid x)$ is the likelihood function or measurement function, this describes the expected measurement $y$ given the state $x$ (if you knew $x$, what measurement $y$ would you expect to see?)

**State Estimation**: Bayes rule naturally leads us to the concept of state estimation, which aims to answer the question poised by Bayes rule: What is the probability of you being in state $x$ given your measured value of the state $y$? We know how to calculate this based on $P(y \mid x)$, $P(x)$, and $P(y)$, but what if we have multiple measurements? This gives rise to the recursive Bayes estimator, which aims to estimate the true state of a dynamical system based on a sequence of observations. At this point, you should be have all of the basics of probability readily available and can move on to applications such as the Recursive Bayes Estimator.
