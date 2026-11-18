'use strict';
'require view';
'require rpc';
'require ui';

var callCrash = rpc.declare({ object: 'gpon', method: 'crash', expect: { '': {} } });

function stamp() {
	return new Date().toISOString().replace(/[:.]/g, '-');
}

return view.extend({
	load: function() {
		return L.resolveDefault(callCrash(), {});
	},

	records: function(text) {
		var out = [], cur = null;
		(text || '').split('\n').forEach(function(line) {
			var m = line.match(/^--- seq=(\d+) time=(\d+)(?: rev=(\S+))?$/);
			if (m) {
				cur = { seq: +m[1], time: +m[2], rev: m[3] || 'unknown', log: '' };
				out.push(cur);
			} else if (/^event /.test(line)) {
				cur = null;
			} else if (cur && line) {
				cur.log += line + '\n';
			}
		});
		out.forEach(function(x) {
			x.kind = /watchdog pretimeout/.test(x.log) ? 'pretimeout' : /Kernel panic/.test(x.log) ? 'panic' : 'oops';
		});
		return out;
	},

	events: function(text) {
		return (text || '').split('\n').filter(function(line) {
			return /^event .* kind=o5loss /.test(line) || /^event .* prev=(unclean|crash) /.test(line);
		}).map(function(line) { return line.replace(/^event /, ''); });
	},

	report: function(r) {
		return [
			'ONU crash report',
			'model:    ' + (r.model || 'unknown'),
			'firmware: ' + (r.release || 'unknown'),
			'kernel:   ' + (r.kernel || 'unknown'),
			'saved:    ' + new Date().toISOString(),
			'',
			r.text || ''
		].join('\n');
	},

	save: function(r) {
		var name = 'crash-report-' + (r.model || 'onu').replace(/[^A-Za-z0-9]+/g, '_') + '-' + stamp() + '.txt';
		var a = E('a', { 'href': URL.createObjectURL(new Blob([ this.report(r) ], { 'type': 'text/plain' })), 'download': name });
		document.body.appendChild(a);
		a.click();
		document.body.removeChild(a);
	},

	render: function(r) {
		var intro = E('p', {}, _('The kernel crashes this unit kept in its own flash: a panic, a watchdog pretimeout or an oops, each with the kernel log before it and the firmware that wrote it; then each restart that did not shut down cleanly and each GPON link loss. If it crashed, download the report and send it to contact@herman-brule.com or attach it to an issue at https://github.com/alphaonex86/openwrt/issues.'));

		if (!r.available)
			return E('div', { 'class': 'cbi-map' }, [
				E('h2', {}, _('Crash')), intro,
				E('p', { 'class': 'alert-message' }, _('This unit keeps no crash records: its flash has no crashlog partition.'))
			]);

		var recs = this.records(r.text);
		var off = / recorder=off/.test(r.text || '') ? E('p', { 'class': 'alert-message' }, _('This boot records no crash: it did not boot from its own flash bank, which it never writes then.')) : '';
		var rows = recs.map(function(x) {
			return E('tr', { 'class': 'tr' }, [
				E('td', { 'class': 'td' }, String(x.seq)),
				E('td', { 'class': 'td' }, E('strong', {}, x.kind.toUpperCase())),
				E('td', { 'class': 'td' }, new Date(x.time * 1000).toISOString()),
				E('td', { 'class': 'td' }, x.rev),
				E('td', { 'class': 'td' }, E('details', {}, [ E('summary', {}, x.log.split('\n')[0]), E('pre', {}, x.log) ]))
			]);
		});

		return E('div', { 'class': 'cbi-map' }, [
			E('h2', {}, _('Crash')), intro, off,
			E('p', {}, recs.length ? E('strong', {}, _('%d crash(es) recorded.').format(recs.length)) : _('No crash recorded.')),
			E('button', { 'class': 'cbi-button cbi-button-action', 'click': ui.createHandlerFn(this, 'save', r) }, [ _('Download the crash report') ]),
			E('table', { 'class': 'table' }, [
				E('tr', { 'class': 'tr table-titles' }, [
					E('th', { 'class': 'th' }, _('Record')), E('th', { 'class': 'th' }, _('Kind')),
					E('th', { 'class': 'th' }, _('Time')), E('th', { 'class': 'th' }, _('Firmware')),
					E('th', { 'class': 'th' }, _('Kernel log'))
				])
			].concat(rows)),
			E('h3', {}, _('Unclean restarts and GPON link losses')),
			E('pre', {}, this.events(r.text).join('\n') || _('None recorded.'))
		]);
	},

	handleSaveApply: null,
	handleSave: null,
	handleReset: null
});
