"""接收端不应执行陈旧、乱序、非法或上一会话的命令。"""
import json
import socket
import time
import unittest
from unittest.mock import patch
from forest_nav_control.sim_endpoint import SimEndpoint


class EndpointTests(unittest.TestCase):
    def setUp(self):
        self.endpoint=SimEndpoint(state_port=1,command_port=0)
        self.endpoint.ready=True
        self.sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.destination=self.endpoint.socket.getsockname()

    def tearDown(self):
        self.sender.close()
        self.endpoint.close()

    def send(self,**changes):
        packet={'version':1,'session':self.endpoint.session,'sequence':1,
                'sent_ns':time.monotonic_ns(),'command':[.2,0.,.3]}
        packet.update(changes)
        self.sender.sendto(json.dumps(packet).encode(),self.destination)

    def test_valid_command_survives_delay(self):
        self.send()
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))
        with patch('forest_nav_control.sim_endpoint.time.monotonic',return_value=time.monotonic()+1):
            self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))

    def test_other_session_is_rejected(self):
        self.send(session='previous-simulation')
        self.assertEqual(self.endpoint.read_command(),(0.,0.,0.))

    def test_delayed_packet_is_accepted(self):
        self.send(sent_ns=time.monotonic_ns()-1000000000)
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))

    def test_replay_does_not_change_command(self):
        self.send(sequence=2)
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))
        self.send(sequence=1,command=[.1,0.,0.])
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))

    def test_nan_rejected_but_finite_speed_is_unrestricted(self):
        self.send(command=[float('nan'),0.,0.])
        self.assertEqual(self.endpoint.read_command(),(0.,0.,0.))
        self.send(command=[4.,-2.,3.])
        self.assertEqual(self.endpoint.read_command(),(4.,-2.,3.))

    def test_ready_report_does_not_override_command(self):
        self.send()
        self.endpoint.ready=False
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))

    def test_non_object_payload_does_not_crash(self):
        self.sender.sendto(b'[]',self.destination)
        self.assertEqual(self.endpoint.read_command(),(0.,0.,0.))

    def test_path_update_and_clear(self):
        self.send(sequence=1,path_revision=1,path=[[0.,0.],[1.,.5]])
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))
        self.assertEqual(self.endpoint.read_path(),
                         (((0.,0.),(1.,.5)),1))
        # 同一修订号的重发仍可携带新速度，但不会重复改路线。
        self.send(sequence=2,path_revision=1,path=[[9.,9.]])
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))
        self.assertEqual(self.endpoint.read_path()[0],((0.,0.),(1.,.5)))
        self.send(sequence=3,path_revision=2,path=[])
        self.assertEqual(self.endpoint.read_command(),(.2,0.,.3))
        self.assertEqual(self.endpoint.read_path(),((),2))

    def test_invalid_path_does_not_replace_command_or_route(self):
        self.send(sequence=1,path_revision=1,path=[[0.,0.],[float('nan'),1.]])
        self.assertEqual(self.endpoint.read_command(),(0.,0.,0.))
        self.assertEqual(self.endpoint.read_path(),((),-1))

    def test_long_route_is_bounded_and_does_not_refresh_command_watchdog(self):
        from forest_nav_control.sim_endpoint import path_packet,MAX_DATAGRAM_BYTES
        points=[[i*.123456789,i*.987654321] for i in range(4096)]
        data=path_packet(points,session=self.endpoint.session,revision=1)
        assert len(data)<=MAX_DATAGRAM_BYTES
        self.sender.sendto(data,self.destination)
        assert self.endpoint.read_command()==(0.,0.,0.)
        path,revision=self.endpoint.read_path()
        assert revision==1 and len(path)<len(points)
        assert path[0]==tuple(points[0]) and path[-1]==tuple(points[-1])
        assert self.endpoint.command_received is None and self.endpoint.command_sequence==-1
        self.send(sequence=1)
        assert self.endpoint.read_command()==(.2,0.,.3)

    def test_invalid_display_does_not_block_following_speed_packet(self):
        p=dict(version=1,kind='path',session=self.endpoint.session,sent_ns=time.monotonic_ns(),
               path_revision=1,path=[[float('nan'),0.]])
        self.sender.sendto(json.dumps(p).encode(),self.destination)
        self.send(sequence=1)
        assert self.endpoint.read_command()==(.2,0.,.3)
        assert self.endpoint.read_path()==((),-1)

    def test_display_clear_and_old_session_are_isolated_from_speed(self):
        from forest_nav_control.sim_endpoint import path_packet
        self.sender.sendto(path_packet([[0,0],[1,1]],session='old',revision=99),self.destination)
        self.endpoint.read_command();assert self.endpoint.read_path()==((),-1)
        self.sender.sendto(path_packet([[0,0],[1,1]],session=self.endpoint.session,revision=2),self.destination)
        self.endpoint.read_command();assert self.endpoint.read_path()[1]==2
        self.sender.sendto(path_packet([],session=self.endpoint.session,revision=1),self.destination)
        self.endpoint.read_command();assert self.endpoint.read_path()[1]==2
        self.sender.sendto(path_packet([],session=self.endpoint.session,revision=3),self.destination)
        self.endpoint.read_command();assert self.endpoint.read_path()==((),3)


if __name__=='__main__':
    unittest.main()
