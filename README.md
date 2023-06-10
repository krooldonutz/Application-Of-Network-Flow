# Application-Of-Network-Flow

Application of Network Flow in a graph using Ford-Fulkerson.

You are the system administrator responsible for processing the backups of your company and
the data needs to be backed up in different data centres. During certain times of the day you
have total control over your company’s data centres and their network connections, but you
need to process the backup requests as fast as possible.
Your company has D data centres represented by 0, 1, . . . , |D| − 1. And you have a list
connections of the direct communication channels between the data centres. connections is
a list of tuples (a, b, t) where:
• a ∈ {0, 1, . . . , |D|−1} is the ID of the data centre from which the communication channel
departs.
• b ∈ {0, 1, . . . , |D| − 1} is the ID of the data centre to which the communication channel
arrives.
• t is a positive integer representing the maximum throughput of that channel.
