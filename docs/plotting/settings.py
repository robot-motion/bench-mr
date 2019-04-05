#!/usr/bin/env python3

PLANNERS = [
    "RRTstar",
    "SORRTstar"
    "ThetaStar",
    "RRTsharp",
    "PRM",
    "Chomp"
]


def settings_table(benchmarks: list):
    def dir2table(d):
        table = "<table>"
        for k, v in d.items():
            table += '<tr><td style="text-align:right !important"><b>%s</b></td>' % k
            if isinstance(v, dict):
                table += '<td>' + dir2table(v) + '</td>'
            else:
                table += '<td style="color:#030;text-align:left !important;font-family:monospace !important" align="left">%s</td>' % v
            table += '</tr>'
        table += "</table>"
        return table

    def comparison_table(data: list, address="settings"):
        def retrieve(benchmark_id, ad):
            ad = ad.split('/')
            result = data[benchmark_id][ad[0]]
            for entry in ad[1:]:
                result = result[entry]
            return result

        table = "<table>"
        d = retrieve(0, address)
        dicts = []
        table += '<tr style="background-color:#003 !important; color:#fff !important"><th style="text-align:left !important">%s</th>' % address + "".join(
            '<th style="text-align:left !important">BM %i</th>' % i for i in range(len(data))) + '</tr>'

        for k, v in d.items():
            curr_address = address + '/' + k
            if isinstance(v, dict):
                dicts.append((k, curr_address, v))
            else:
                table += '<tr><td style="text-align:right !important"><b>%s</b></td>' % k
                for bm in range(len(data)):
                    bv = retrieve(bm, curr_address)
                    table += '<td style="'
                    if v != bv:
                        table += 'color:#069 !important;background-color:#ff4 !important;'
                    table += 'text-align:left !important;font-family:monospace !important" align="left">%s</td>' % bv

            table += '</tr>'
        for k, a, v in dicts:
            table += '<tr><td style="text-align:right !important"><b>%s</b></td>' % k
            table += '<td colspan="%i">' % (len(data)) + comparison_table(data, a) + '</td></tr>'

        table += "</table>"
        return table

    if len(benchmarks) == 0:
        return '<table>%s</table>' % dir2table(benchmarks[0]["settings"])
    else:
        return comparison_table(benchmarks, "settings")
