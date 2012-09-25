../../bzrflag/bin/bzrflag --world=../../bzrflag/maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &
sleep 2
pypy src/really_dumb_agent.py localhost 50100 &
python src/really_dumb_agent.py localhost 50101 &
#python src/agent0.py localhost 50101 &
#python bzagents/agent0.py localhost 50102 &
#python bzagents/agent0.py localhost 50103 &