#include "ServerAPI.h"

#include <stdexcept>

#include "TCPConnection.h"

ServerAPI::ServerAPI()
	: _all_clients_connected(false)
	, _running(true)
	, _clients{}
	, _clients_amount(0)
	, _accept_clients_thread(&ServerAPI::_acceptClients, this) {}

ServerAPI::~ServerAPI(){
	_running = false;

	for (int i = 0; i < MAX_CLIENTS; i++){
		if (_clients[i] != NULL){
			delete _clients[i];
		}
	}

	_accept_clients_thread.join();
}

bool ServerAPI::isAbleToDeliverData(){
	int check = 0;
	int ref = 0;
	for(int i = 0; i < MAX_CLIENTS; i++){
		ref |= 1 << i;

		if (_clients[i] != NULL && _clients[i]->isActive()) {
			check |= _clients[i]->isDataAvailable() << i;
		}
	}

	return _all_clients_connected && (ref == check);
}

Client* ServerAPI::getClient(int index){
	if (index < 0 || index >= MAX_CLIENTS || _clients[index] == NULL){
		cout << "error in " << index << endl;
		throw std::invalid_argument("illegal index in getClient");
	}

	return (_clients[index]);
}

void ServerAPI::_acceptClients(){
	int tcp_socket = 0;

	TCPConnection con;
	if (con.createConnection(SERVER, CONNECTION_PORT, "") < 0){
		LOG_ERROR << "Failed to create tcp socket" << endl;
		_running = false;
		return;
	}
	con.setNonBlocking();

	while (_running){
		for (int i = 0; i < _clients_amount; i++){
			if (_clients[i] != NULL && !_clients[i]->isActive()){
				LOG_WARNING << "Node " << i << " disconnected" << endl;
				if (_clients[i]){
					delete _clients[i];
				}

				_clients_amount--;
				_all_clients_connected = 0;
			}
		}

		if (_clients_amount < MAX_CLIENTS){
			tcp_socket = con.acceptConnection();
			if (tcp_socket >= 0){
				int pos = 0;

				for (; pos < MAX_CLIENTS; pos++){
					if (_clients[pos] == NULL){
						break;
					}
				}

				_clients[pos] = new Client(pos, tcp_socket);

				if (++_clients_amount == MAX_CLIENTS){
					_all_clients_connected = 1;
				}
			}
		}

		usleep(100000); //sleep for 1s to give main thread time
	}

	con.closeConnection();
}
