#../../bzrflag/bin/bzrflag --world=../../bzrflag/maps/rotated_box_world.bzw --friendly-fire --red-port=50100 --purple-port=50101 --green-port=50102 --blue-port=50103 $@ &
../../bzrflag/bin/bzrflag --world=../../bzrflag/maps/four_ls.bzw --friendly-fire --red-port=50100 --green-port=50101 --purple-port=50102 --blue-port=50103 $@ &

sleep 2

echo "working"

#make --directory=src/

#src/compiled/really_dumb_agent.pyc localhost 50100 &

#pypy src/pfpdagent.py localhost 50100 &
pypy src/pfagent.py localhost 50101 &
#pypy src/pfpdagent.py localhost 50102 &
#pypy src/pfpdagent.py localhost 50103 &
#pypy ../../bzrflag/bzagents/agent0.py localhost 50101 &
pypy src/really_dumb_agent.py localhost 50100 &
#pypy src/pfagent.py localhost 50102 &
#pypy src/pfagent.py localhost 50103 &
#pypy ../../bzrflag/bzagents/agent0.py localhost 50103 &




#pypy src/pfagent.py localhost 50101 &
#pypy src/attractive_field_agent.py localhost 50101 &
#pypy src/really_dumb_agent.py localhost 50103 &
#pypy src/pfagent.py localhost 50101 &
#pypy src/really_dumb_agent.py localhost 50102 &
#pypy src/really_dumb_agent.py localhost 50102 &
#pypy src/really_dumb_agent.py localhost 50103 &
#python src/agent0.py localhost 50101 &
#python bzagents/agent0.py localhost 50102 &
#python bzagents/agent0.py localhost 50103 &