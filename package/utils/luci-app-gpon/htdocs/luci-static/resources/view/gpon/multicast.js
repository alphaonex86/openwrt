'use strict';
'require view';
'require form';
'require rpc';
'require uci';

var callInitList = rpc.declare({ object: 'luci', method: 'getInitList', params: [ 'name' ], expect: { '': {} } });
var callInitAction = rpc.declare({ object: 'luci', method: 'setInitAction', params: [ 'name', 'action' ], expect: { result: false } });

function initAction(name, action) {
	return callInitAction(name, action).then(function(ok) {
		if (!ok)
			throw new Error(_('%s %s failed').format(name, action));
	});
}

return view.extend({
	load: function() {
		return Promise.all([ L.resolveDefault(callInitList('igmpproxy'), {}), uci.load('igmpproxy') ]);
	},

	render: function(data) {
		var proxy = (data[0] || {}).igmpproxy || {};
		var m = new form.Map('igmpproxy', _('Multicast (IPTV)'),
			_('The ONU joins the multicast groups (IPTV channels) its LAN clients ask for and forwards them from the WAN.'));
		var s = m.section(form.TypedSection, 'igmpproxy');
		s.anonymous = true;

		var o = s.option(form.Flag, '_proxy', _('Multicast'),
			_('Off: no group reaches the LAN, whatever a client asks for.'));
		o.rmempty = false;
		o.cfgvalue = function() { return proxy.enabled ? '1' : '0'; };
		o.write = function(sid, value) {
			return value == '1'
				? initAction('igmpproxy', 'enable').then(function() { return initAction('igmpproxy', 'start'); })
				: initAction('igmpproxy', 'stop').then(function() { return initAction('igmpproxy', 'disable'); });
		};

		o = s.option(form.Flag, 'wired_only', _('Wired LAN ports only'),
			_('WiFi stations get no multicast. Where the ONU\'s switch routes multicast, it then carries every group and IPTV costs no CPU; with WiFi members a group stays on the CPU, the only path to the radio.'));
		o.default = '0';
		o.depends('_proxy', '1');

		return m.render();
	}
});
