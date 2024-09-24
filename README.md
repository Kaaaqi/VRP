-run_mode    -node_mode    -run_times    -start_idx    -end_idx

[run_mode]: all /  choose 

    all: run all the instances

    choose: run the chosen instances

[ node_mode ]: normal

    normal: do pretreatment process

[ run_times ]: let each instance run for run_times times

[ start_idx ]: valid when run_mode is choose.  Represent the first chosen instance's index 

[ end_idx ]: valid when run_mode is choose. Represent the last chosen instance's index





.data/  folder stores all the results

.Instance/ folder stores all the instances 


