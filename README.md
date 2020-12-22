Timmer: A timmer bese on windows sys could be used at record time interval with uncertanty: 1.5e-7 S, allows mult-thread.

Serial: template to tramsmit information with serial com;






Edition 1.2

"Timmer" func might make some mistake with "Sleep" func

Edition 1.3

support string to be the parameters, and support the COM>10

Edition 1.4

add clone timmer func to allows timmer runs independently
add individual func to solve the cose of mutex.

Edition 1.5

mutex synchronize has been optimized


Edition 1.6
reduce mutex, mult-thread cost of timmer.
Serial mult-thread read&write redundancy protect
